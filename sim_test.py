import time
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

        # Commanded velocities
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

    def cmd_vel_callback(self, msg: Twist):
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z


def main():
    rclpy.init()
    node = AMRSimNode()

    # Load the MuJoCo Model
    model = mujoco.MjModel.from_xml_path('my_amr.xml')
    data = mujoco.MjData(model)

    # Cache Sensor and Actuator IDs
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "lidar_dist")
    lidar_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lidar_motor")
    left_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
    right_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")

    step_counter = 0

    print("MuJoCo simulation started. Launching viewer...")
    print("Listening for velocity commands on topic: /cmd_vel")
    print("Publishing LiDAR range data on topic: /sim_lidar")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Process incoming ROS 2 callbacks (e.g. /cmd_vel)
            rclpy.spin_once(node, timeout_sec=0.0)

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

            # Step the physics engine & update viewer
            mujoco.mj_step(model, data)
            viewer.sync()

            step_counter += 1

            # Publish LiDAR Range message at ~10Hz (50 physics steps * 0.002s = 0.1s)
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
