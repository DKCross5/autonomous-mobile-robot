#!/usr/bin/env python3
import os
import time
import math
import threading
import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
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

# ── LiDAR Sensor Parameters (ROBOTIS LDS-01 spec) ─────────────────────────────
LIDAR_NUM_BEAMS    = 360        # 360 samples per revolution (1° angular resolution)
LIDAR_ANGLE_MIN    = 0.0        # radians – scan start angle (facing +X in lidar_link)
LIDAR_ANGLE_MAX    = 2.0 * math.pi  # radians – scan end angle
LIDAR_ANGLE_INC    = (2.0 * math.pi) / LIDAR_NUM_BEAMS  # radians per beam
LIDAR_RANGE_MIN    = 0.12       # metres – LDS-01 minimum reliable detection range
LIDAR_RANGE_MAX    = 3.50       # metres – LDS-01 maximum detection range
LIDAR_SCAN_HZ      = 25.0       # Hz – scan publication rate (25Hz for high-speed tracking)
# Height of the lidar_beam site above the floor, used to anchor each ray origin
LIDAR_MOUNT_X      = 0.05       # metres forward from chassis centre (matches XML)
LIDAR_MOUNT_Z      = 0.09       # metres above chassis centre (matches XML)
# ──────────────────────────────────────────────────────────────────────────────


class AMRSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        # 1. Telemetry & Control Publishers / Subscribers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
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

        # 2. base_link -> lidar_link (Mount offset pos=[0.05, 0, 0.09], fixed forward orientation)
        t_lidar = TransformStamped()
        t_lidar.header.stamp = stamp
        t_lidar.header.frame_id = 'base_link'
        t_lidar.child_frame_id = 'lidar_link'
        t_lidar.transform.translation.x = 0.05
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.09

        # Rigidly mounted sensor frame: zero rotation relative to chassis (REP-105 / REP-117)
        # Laser rays are indexed relative to lidar_link (+X forward), so the frame must stay static!
        t_lidar.transform.rotation.w = 1.0
        t_lidar.transform.rotation.x = 0.0
        t_lidar.transform.rotation.y = 0.0
        t_lidar.transform.rotation.z = 0.0

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


    def synthesize_laserscan(self, model: 'mujoco.MjModel', data: 'mujoco.MjData',
                              chassis_body_id: int, lidar_body_id: int, stamp) -> LaserScan:
        """
        Synthesise a full 360-beam planar LaserScan using MuJoCo's mj_ray() API.

        Strategy (6-DOF Geometric Fidelity)
        ----------------------------------
        To eliminate micro-jitter during aggressive maneuvers (braking, turning,
        caster swiveling), the rays are fired directly from the exact world-frame
        lidar center (data.xpos[lidar_body_id]) and rotated by the robot chassis's
        actual 3D rotation matrix R = data.xmat[chassis_body_id].

        This guarantees that the simulated rays in MuJoCo follow the exact same
        3D spatial paths that RViz2 reconstructs via the TF tree (odom -> base_link -> lidar_link),
        giving millimeter-accurate wall lock even when the chassis pitches or rolls.
        """
        # 1. Exact 3D ray origin matching lidar_link mount position
        ray_origin = data.xpos[lidar_body_id].copy()

        # 2. Exact 3D rotation matrix of chassis (matches base_link in TF)
        # Note: We use chassis_body_id so the scan does not spin with the visual motor!
        R = data.xmat[chassis_body_id].reshape(3, 3)

        # 3. Cast 360 rays with exact 3D orientation
        ranges = []
        geomid = np.array([-1], dtype=np.int32)   # reused scratch buffer

        for i in range(LIDAR_NUM_BEAMS):
            theta = LIDAR_ANGLE_MIN + i * LIDAR_ANGLE_INC
            # Local beam vector in lidar_link frame (+X forward, +Y left, +Z up)
            local_dir = np.array([math.cos(theta), math.sin(theta), 0.0], dtype=np.float64)
            # Transform to world coordinates using robot's full 3D rotation
            ray_dir = R @ local_dir

            dist = mujoco.mj_ray(model, data,
                                 ray_origin, ray_dir,
                                 None,             # geomgroup mask (None = all)
                                 1,                # flg_static: include static geoms
                                 lidar_body_id,    # bodyexclude: skip the lidar body
                                 geomid)

            # REP-117 range gating: out-of-range or blind-spot -> inf
            if LIDAR_RANGE_MIN <= dist <= LIDAR_RANGE_MAX:
                ranges.append(float(dist))
            else:
                ranges.append(float('inf'))

        # 4. Build and return the LaserScan message
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'lidar_link'

        scan.angle_min       = LIDAR_ANGLE_MIN
        scan.angle_max       = LIDAR_ANGLE_MAX - LIDAR_ANGLE_INC  # last sample (no wrap-around)
        scan.angle_increment = LIDAR_ANGLE_INC
        scan.time_increment  = 0.0  # Instantaneous scan in simulation (REP-117; eliminates rolling-shutter seams)
        scan.scan_time       = 1.0 / LIDAR_SCAN_HZ
        scan.range_min       = LIDAR_RANGE_MIN
        scan.range_max       = LIDAR_RANGE_MAX
        scan.ranges          = ranges
        scan.intensities     = []   # LDS-01 does not publish intensity data

        return scan


def main():
    rclpy.init()
    node = AMRSimNode()

    # Spin ROS 2 callbacks in a separate daemon thread so messages are never dropped
    ros_spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_spin_thread.start()

    # Load the MuJoCo Model
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'my_amr.xml')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Cache Actuator and Body IDs
    lidar_motor_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lidar_motor")
    left_motor_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
    right_motor_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")
    # Body IDs: lidar for exclusion; chassis for 3D orientation matching base_link TF
    lidar_body_id   = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "lidar")
    chassis_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    # Publish the full 360-beam scan at LIDAR_SCAN_HZ (25 Hz).
    # At 500 Hz physics, one scan every 20 steps.
    scan_publish_interval = int(round(1.0 / (LIDAR_SCAN_HZ * model.opt.timestep)))

    step_counter = 0

    # Smoothed velocity states (starts at zero)
    current_linear_x = 0.0
    current_angular_z = 0.0

    print("==========================================================")
    print("MuJoCo simulation started. Launching viewer...")
    print(f"Actuator IDs: left_motor={left_motor_id}, right_motor={right_motor_id}, lidar_motor={lidar_motor_id}")
    print("Active TF Broadcast: odom -> base_link -> lidar_link")
    print(f"LaserScan: {LIDAR_NUM_BEAMS} beams @ {LIDAR_SCAN_HZ:.0f} Hz  ({LIDAR_RANGE_MIN}–{LIDAR_RANGE_MAX} m)")
    print("Publishing Topics: /odom, /robot_visual, /robot_path, /scan")
    print("Listening for velocity commands on topic: /cmd_vel")
    print("==========================================================")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall_start_time = time.time()
        fps_timer = time.time()
        fps_step_count = 0
        fps_sync_count = 0
        fps_scan_count = 0

        while viewer.is_running():
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
            fps_step_count += 1

            # Sync viewer, broadcast TF, Odometry, and 3D Visual Markers at ~100Hz (every 5 physics steps)
            if step_counter % 5 == 0:
                fps_sync_count += 1
                now_msg = node.get_clock().now().to_msg()
                viewer.sync()
                node.publish_transforms(now_msg, data.qpos)
                node.publish_odometry(now_msg, data.qpos, data.qvel)
                node.marker_pub.publish(node.create_robot_markers(now_msg))

            # Publish full 360-beam LaserScan at LIDAR_SCAN_HZ (25 Hz, every 20 steps)
            if step_counter % scan_publish_interval == 0:
                fps_scan_count += 1
                now_msg = node.get_clock().now().to_msg()
                scan_msg = node.synthesize_laserscan(model, data, chassis_body_id, lidar_body_id, now_msg)
                node.scan_pub.publish(scan_msg)

            # High-efficiency display-rate real-time pacing (~100 Hz)
            # Batching sleep to the 5-step sync boundary eliminates 500 OS sleep syscalls/sec,
            # avoiding timer jitter and keeping simulation locked 1:1 with wall-clock time.
            if step_counter % 5 == 0:
                target_wall_time = wall_start_time + step_counter * dt
                now = time.time()
                sleep_time = target_wall_time - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif now - target_wall_time > 0.1:
                    # Prevent catch-up loop if window dragging or WSL scheduler lags
                    wall_start_time = now - step_counter * dt

            # Performance telemetry printout (rolling 1.0-second window)
            now = time.time()
            dt_fps = now - fps_timer
            if dt_fps >= 1.0:
                actual_phys_hz = fps_step_count / dt_fps
                actual_viewer_fps = fps_sync_count / dt_fps
                actual_scan_hz = fps_scan_count / dt_fps
                print(f"\r\033[K[SIM TELEMETRY] Physics: {actual_phys_hz:.1f} Hz | MuJoCo Viewer: {actual_viewer_fps:.1f} FPS | LaserScan: {actual_scan_hz:.1f} Hz", end='', flush=True)
                fps_timer = now
                fps_step_count = 0
                fps_sync_count = 0
                fps_scan_count = 0

    # Clean shutdown
    print("Closing simulation node...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
