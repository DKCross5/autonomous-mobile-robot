import time
import threading
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

# Robot Physical Parameters (Differential Drive)
# Wheels: ~6-inch diameter rubber wheels -> radius ~ 0.0762 m
# Wheelbase (track width between left and right wheels): 0.36 m
WHEEL_RADIUS = 0.0762
WHEEL_BASE = 0.36

# Two-Stage Watchdog Timing (Deadman's Switch):
# 1. FIRST_PRESS_TIMEOUT (0.70s): Bridges the OS keyboard typamatic repeat delay (~500ms)
#    when a key is first pressed, eliminating the mid-press stop/start stutter.
# 2. HOLD_TIMEOUT (0.25s): Once key auto-repeat is established, provides crisp,
#    immediate stopping within 250ms when the key is released.
FIRST_PRESS_TIMEOUT = 0.70
HOLD_TIMEOUT = 0.25

# Acceleration Limits (m/s^2 and rad/s^2) for fluid, jitter-free motion
MAX_LIN_ACCEL = 3.0   # Reaches full speed (0.5 m/s) in ~0.16s (instant feel, zero jerk)
MAX_ANG_ACCEL = 8.0   # Reaches full rotation (1.0 rad/s) in ~0.12s


class AMRSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        # 1. LiDAR Range Publisher (broadcasting on '/sim_lidar')
        self.lidar_pub = self.create_publisher(Range, 'sim_lidar', 10)

        # 2. Velocity Command Subscriber (listening on '/cmd_vel')
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Commanded velocities & timing state
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_cmd_time = 0.0
        self.repeat_mode = False

    def cmd_vel_callback(self, msg: Twist):
        now = time.time()
        # If successive commands arrive within 0.20s, key repeat stream is actively held
        if 0.0 < (now - self.last_cmd_time) < 0.20:
            self.repeat_mode = True
        else:
            self.repeat_mode = False

        if (msg.linear.x != self.target_linear_x or msg.angular.z != self.target_angular_z):
            self.get_logger().info(
                f"Active Command -> linear.x: {msg.linear.x:.2f} m/s, angular.z: {msg.angular.z:.2f} rad/s"
            )

        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        self.last_cmd_time = now


def main():
    rclpy.init()
    node = AMRSimNode()

    # Spin ROS 2 callbacks in a separate daemon thread so messages are never dropped or starved
    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spin_thread.start()

    # Load the MuJoCo Model
    model = mujoco.MjModel.from_xml_path('my_amr.xml')
    data = mujoco.MjData(model)

    # Cache Sensor and Actuator IDs
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_dist")
    lidar_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lidar_motor")
    left_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
    right_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")

    step_counter = 0

    # Smoothed velocity states (starts at zero)
    current_linear_x = 0.0
    current_angular_z = 0.0

    print("==========================================================")
    print("MuJoCo simulation started. Launching viewer...")
    print(f"Actuator IDs: left_motor={left_motor_id}, right_motor={right_motor_id}, lidar_motor={lidar_motor_id}")
    print("Fluid Motion Control: Two-Stage Watchdog + Slew Rate Acceleration Active")
    print("Listening for velocity commands on topic: /cmd_vel")
    print("Publishing LiDAR range data on topic: /sim_lidar")
    print("==========================================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            dt = model.opt.timestep

            # Two-Stage Watchdog:
            # Uses FIRST_PRESS_TIMEOUT (0.7s) to absorb OS keyboard repeat delay,
            # switching to HOLD_TIMEOUT (0.25s) once the key repeat stream is established.
            timeout = HOLD_TIMEOUT if node.repeat_mode else FIRST_PRESS_TIMEOUT
            if time.time() - node.last_cmd_time > timeout:
                if node.target_linear_x != 0.0 or node.target_angular_z != 0.0:
                    node.target_linear_x = 0.0
                    node.target_angular_z = 0.0
                    node.repeat_mode = False

            # Slew-Rate Acceleration Limiting (Instant response, zero physics jerk)
            dv = node.target_linear_x - current_linear_x
            max_dv = MAX_LIN_ACCEL * dt
            if dv > max_dv:
                current_linear_x += max_dv
            elif dv < -max_dv:
                current_linear_x -= max_dv
            else:
                current_linear_x = node.target_linear_x

            dw = node.target_angular_z - current_angular_z
            max_dw = MAX_ANG_ACCEL * dt
            if dw > max_dw:
                current_angular_z += max_dw
            elif dw < -max_dw:
                current_angular_z -= max_dw
            else:
                current_angular_z = node.target_angular_z

            # Differential Drive Kinematics
            v_left = current_linear_x - (current_angular_z * WHEEL_BASE / 2.0)
            v_right = current_linear_x + (current_angular_z * WHEEL_BASE / 2.0)

            omega_left = v_left / WHEEL_RADIUS
            omega_right = v_right / WHEEL_RADIUS

            # Apply motor controls
            data.ctrl[left_motor_id] = omega_left
            data.ctrl[right_motor_id] = omega_right
            data.ctrl[lidar_motor_id] = 10.0  # Constant spin for LiDAR

            # Step the physics engine (500Hz, dt = 0.002s)
            mujoco.mj_step(model, data)

            step_counter += 1

            # Sync viewer at ~50Hz (every 10 physics steps) to prevent software rendering from freezing CPU
            if step_counter % 10 == 0:
                viewer.sync()

            # Publish LiDAR Range message at ~10Hz (every 50 physics steps)
            if step_counter % 50 == 0:
                distance = data.sensordata[sensor_id]

                msg = Range()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = "lidar_link"
                msg.radiation_type = Range.INFRARED
                msg.min_range = 0.0
                msg.max_range = 5.0
                msg.field_of_view = 0.1

                # If rangefinder does not hit anything (>0 check), use 4.9m to avoid RViz2 out-of-bounds dropping
                if distance > 0:
                    msg.range = float(distance)
                else:
                    msg.range = 4.9

                node.lidar_pub.publish(msg)

            # Pace physics loop to real-time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # Clean shutdown
    print("Closing simulation node...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
