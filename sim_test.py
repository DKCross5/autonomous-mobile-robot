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

# Command watchdog timeout (Deadman's switch in seconds).
# If no key is actively pressed within this duration, the robot automatically stops.
CMD_TIMEOUT = 0.3


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

        # Commanded velocities & timestamp for press-and-hold watchdog
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_cmd_time = 0.0

    def cmd_vel_callback(self, msg: Twist):
        # Only log when velocity values change to avoid spamming the console on key hold
        if (msg.linear.x != self.target_linear_x or msg.angular.z != self.target_angular_z):
            self.get_logger().info(
                f"Active Command -> linear.x: {msg.linear.x:.2f} m/s, angular.z: {msg.angular.z:.2f} rad/s"
            )

        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        self.last_cmd_time = time.time()


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

    print("==========================================================")
    print("MuJoCo simulation started. Launching viewer...")
    print(f"Actuator IDs: left_motor={left_motor_id}, right_motor={right_motor_id}, lidar_motor={lidar_motor_id}")
    print(f"Deadman's Switch Watchdog: Active ({CMD_TIMEOUT}s timeout)")
    print("Listening for velocity commands on topic: /cmd_vel")
    print("Publishing LiDAR range data on topic: /sim_lidar")
    print("==========================================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Press-and-Hold Watchdog (Deadman's Switch):
            # If no command received within CMD_TIMEOUT, automatically decelerate to stop
            if time.time() - node.last_cmd_time > CMD_TIMEOUT:
                if node.target_linear_x != 0.0 or node.target_angular_z != 0.0:
                    node.get_logger().info("Key released -> Stopping robot.")
                    node.target_linear_x = 0.0
                    node.target_angular_z = 0.0

            # Differential Drive Kinematics
            # v_left = v - (omega * L / 2)
            # v_right = v + (omega * L / 2)
            # wheel_omega = v_wheel / radius
            v = node.target_linear_x
            w = node.target_angular_z

            v_left = v - (w * WHEEL_BASE / 2.0)
            v_right = v + (w * WHEEL_BASE / 2.0)

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
