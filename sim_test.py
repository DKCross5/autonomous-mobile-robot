import time
import math
import threading
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

# Robot Physical Parameters (Differential Drive)
WHEEL_RADIUS = 0.0762  # 6-inch diameter rubber wheels -> radius ~ 0.0762 m
WHEEL_BASE = 0.36      # 0.36 m distance between left and right wheels

# Two-Stage Watchdog Timing (Deadman's Switch)
FIRST_PRESS_TIMEOUT = 0.55
HOLD_TIMEOUT = 0.15

# Dynamic Slew Rates (m/s^2 and rad/s^2)
STEER_LIN_ACCEL = 4.0
STEER_ANG_ACCEL = 10.0
BRAKE_LIN_DECEL = 2.0
BRAKE_ANG_DECEL = 5.0


class AMRSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        # 1. Telemetry & Control Publishers / Subscribers
        self.lidar_pub = self.create_publisher(Range, 'sim_lidar', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'robot_visual', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Dynamic TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Commanded velocities & timing state
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_cmd_time = 0.0
        self.repeat_mode = False

        # Breadcrumb Path History
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.last_path_x = 0.0
        self.last_path_y = 0.0

    def cmd_vel_callback(self, msg: Twist):
        now = time.time()
        if 0.0 < (now - self.last_cmd_time) < 0.15:
            self.repeat_mode = True
        else:
            self.repeat_mode = False

        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.repeat_mode = False

        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        self.last_cmd_time = now

    def publish_transforms(self, stamp, qpos):
        # 1. odom -> base_link
        t_base = TransformStamped()
        t_base.header.stamp = stamp
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_link'
        t_base.transform.translation.x = float(qpos[0])
        t_base.transform.translation.y = float(qpos[1])
        t_base.transform.translation.z = float(qpos[2])
        # MuJoCo quat is (w, x, y, z), ROS quat is (x, y, z, w)
        t_base.transform.rotation.w = float(qpos[3])
        t_base.transform.rotation.x = float(qpos[4])
        t_base.transform.rotation.y = float(qpos[5])
        t_base.transform.rotation.z = float(qpos[6])

        # 2. base_link -> lidar_link (Mount offset pos=[0.05, 0, 0.09], spinning with lidar_spin angle)
        t_lidar = TransformStamped()
        t_lidar.header.stamp = stamp
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        t_lidar.transform.translation.x = 0.05
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.09

        # Rotation around Z by lidar_spin joint angle (qpos[9])
        lidar_angle = float(qpos[9])
        t_lidar.transform.rotation.w = math.cos(lidar_angle / 2.0)
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = math.sin(lidar_angle / 2.0)

        self.tf_broadcaster.sendTransform([t_base, t_lidar])

    def publish_odometry(self, stamp, qpos, qvel):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose in odom frame
        odom.pose.pose.position.x = float(qpos[0])
        odom.pose.pose.position.y = float(qpos[1])
        odom.pose.pose.position.z = float(qpos[2])
        odom.pose.pose.orientation.w = float(qpos[3])
        odom.pose.pose.orientation.x = float(qpos[4])
        odom.pose.pose.orientation.y = float(qpos[5])
        odom.pose.pose.orientation.z = float(qpos[6])

        # Convert world velocities to base_link body frame
        w, x, y, z = qpos[3], qpos[4], qpos[5], qpos[6]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        vx_world, vy_world = float(qvel[0]), float(qvel[1])
        odom.twist.twist.linear.x = vx_world * math.cos(yaw) + vy_world * math.sin(yaw)
        odom.twist.twist.linear.y = -vx_world * math.sin(yaw) + vy_world * math.cos(yaw)
        odom.twist.twist.angular.z = float(qvel[5])

        self.odom_pub.publish(odom)

        # Update Path trail if moved > 3cm
        dist_from_last = math.hypot(qpos[0] - self.last_path_x, qpos[1] - self.last_path_y)
        if dist_from_last > 0.03:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.path_msg.poses.append(pose)
            self.last_path_x = float(qpos[0])
            self.last_path_y = float(qpos[1])
            if len(self.path_msg.poses) > 500:
                self.path_msg.poses.pop(0)
            self.path_pub.publish(self.path_msg)

    def create_robot_markers(self, stamp):
        """Creates a detailed 3D visual representation of the AMR and obstacle in RViz2."""
        markers = MarkerArray()

        # 1. Blue Trapezoidal Chassis Deck
        m_chassis = Marker()
        m_chassis.header.stamp = stamp
        m_chassis.header.frame_id = 'base_link'
        m_chassis.ns = 'amr_chassis'
        m_chassis.id = 0
        m_chassis.type = Marker.CUBE
        m_chassis.action = Marker.ADD
        m_chassis.pose.position.x = -0.04
        m_chassis.pose.position.y = 0.0
        m_chassis.pose.position.z = 0.02
        m_chassis.pose.orientation.w = 1.0
        m_chassis.scale.x = 0.40
        m_chassis.scale.y = 0.28
        m_chassis.scale.z = 0.08
        m_chassis.color.r = 0.05
        m_chassis.color.g = 0.45
        m_chassis.color.b = 0.90
        m_chassis.color.a = 0.95
        markers.markers.append(m_chassis)

        # 2. Left Drive Wheel (Rubber cylinder)
        m_lwheel = Marker()
        m_lwheel.header.stamp = stamp
        m_lwheel.header.frame_id = 'base_link'
        m_lwheel.ns = 'amr_wheels'
        m_lwheel.id = 1
        m_lwheel.type = Marker.CYLINDER
        m_lwheel.action = Marker.ADD
        m_lwheel.pose.position.x = 0.0
        m_lwheel.pose.position.y = 0.18
        m_lwheel.pose.position.z = 0.0
        # Rotate cylinder axis from Z to Y
        m_lwheel.pose.orientation.w = 0.7071
        m_lwheel.pose.orientation.x = 0.7071
        m_lwheel.scale.x = 0.1524
        m_lwheel.scale.y = 0.1524
        m_lwheel.scale.z = 0.04
        m_lwheel.color.r = 0.15
        m_lwheel.color.g = 0.15
        m_lwheel.color.b = 0.15
        m_lwheel.color.a = 1.0
        markers.markers.append(m_lwheel)

        # 3. Right Drive Wheel
        m_rwheel = Marker()
        m_rwheel.header.stamp = stamp
        m_rwheel.header.frame_id = 'base_link'
        m_rwheel.ns = 'amr_wheels'
        m_rwheel.id = 2
        m_rwheel.type = Marker.CYLINDER
        m_rwheel.action = Marker.ADD
        m_rwheel.pose.position.x = 0.0
        m_rwheel.pose.position.y = -0.18
        m_rwheel.pose.position.z = 0.0
        m_rwheel.pose.orientation.w = 0.7071
        m_rwheel.pose.orientation.x = 0.7071
        m_rwheel.scale.x = 0.1524
        m_rwheel.scale.y = 0.1524
        m_rwheel.scale.z = 0.04
        m_rwheel.color.r = 0.15
        m_rwheel.color.g = 0.15
        m_rwheel.color.b = 0.15
        m_rwheel.color.a = 1.0
        markers.markers.append(m_rwheel)

        # 4. Rear Swivel Caster
        m_caster = Marker()
        m_caster.header.stamp = stamp
        m_caster.header.frame_id = 'base_link'
        m_caster.ns = 'amr_caster'
        m_caster.id = 3
        m_caster.type = Marker.SPHERE
        m_caster.action = Marker.ADD
        m_caster.pose.position.x = -0.22
        m_caster.pose.position.y = 0.0
        m_caster.pose.position.z = -0.0412
        m_caster.pose.orientation.w = 1.0
        m_caster.scale.x = 0.07
        m_caster.scale.y = 0.07
        m_caster.scale.z = 0.07
        m_caster.color.r = 0.7
        m_caster.color.g = 0.7
        m_caster.color.b = 0.7
        m_caster.color.a = 1.0
        markers.markers.append(m_caster)

        # 5. Obstacle Wall (Red box in odom frame at x=2.5)
        m_wall = Marker()
        m_wall.header.stamp = stamp
        m_wall.header.frame_id = 'odom'
        m_wall.ns = 'environment'
        m_wall.id = 4
        m_wall.type = Marker.CUBE
        m_wall.action = Marker.ADD
        m_wall.pose.position.x = 2.5
        m_wall.pose.position.y = 0.0
        m_wall.pose.position.z = 0.5
        m_wall.pose.orientation.w = 1.0
        m_wall.scale.x = 0.2
        m_wall.scale.y = 2.0
        m_wall.scale.z = 1.0
        m_wall.color.r = 0.95
        m_wall.color.g = 0.15
        m_wall.color.b = 0.15
        m_wall.color.a = 0.75
        markers.markers.append(m_wall)

        return markers


def main():
    rclpy.init()
    node = AMRSimNode()

    # Spin ROS 2 callbacks in a separate daemon thread so messages are never dropped
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
    print("Active TF Broadcast: odom -> base_link -> lidar_link")
    print("Publishing Topics: /odom, /robot_visual, /robot_path, /sim_lidar")
    print("Listening for velocity commands on topic: /cmd_vel")
    print("==========================================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            dt = model.opt.timestep

            # Two-Stage Watchdog
            timeout = HOLD_TIMEOUT if node.repeat_mode else FIRST_PRESS_TIMEOUT
            if time.time() - node.last_cmd_time > timeout:
                if node.target_linear_x != 0.0 or node.target_angular_z != 0.0:
                    node.target_linear_x = 0.0
                    node.target_angular_z = 0.0
                    node.repeat_mode = False

            # Linear Slew-Rate Limiter
            dv = node.target_linear_x - current_linear_x
            is_full_stop_linear = (node.target_linear_x == 0.0 and node.target_angular_z == 0.0)
            rate_linear = BRAKE_LIN_DECEL if is_full_stop_linear else STEER_LIN_ACCEL
            max_dv = rate_linear * dt
            if dv > max_dv:
                current_linear_x += max_dv
            elif dv < -max_dv:
                current_linear_x -= max_dv
            else:
                current_linear_x = node.target_linear_x

            # Angular Slew-Rate Limiter
            dw = node.target_angular_z - current_angular_z
            is_full_stop_angular = (node.target_linear_x == 0.0 and node.target_angular_z == 0.0)
            rate_angular = BRAKE_ANG_DECEL if is_full_stop_angular else STEER_ANG_ACCEL
            max_dw = rate_angular * dt
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

            # Sync viewer at ~50Hz (every 10 physics steps)
            if step_counter % 10 == 0:
                viewer.sync()

            # Dynamic TF & Odometry Broadcast (~50Hz, every 10 physics steps)
            if step_counter % 10 == 0:
                now_msg = node.get_clock().now().to_msg()
                node.publish_transforms(now_msg, data.qpos)
                node.publish_odometry(now_msg, data.qpos, data.qvel)

            # 3D Visual Robot Markers & Path Broadcast (~20Hz, every 25 steps)
            if step_counter % 25 == 0:
                now_msg = node.get_clock().now().to_msg()
                node.marker_pub.publish(node.create_robot_markers(now_msg))

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
