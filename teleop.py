#!/usr/bin/env python3
import sys
import os
import fcntl
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
==============================================================
         AMR Dual-Axis Real-Time Teleop Node
==============================================================
Drive Controls:
--------------------------------------------------------------
  Multi-Key Steering (Add turning while driving):
    - Hold Forward (W / Up), then add Left (A) or Right (D)
      -> Smoothly transitions into Forward-Left or Forward-Right arc!
    - Hold Reverse (S / Down), then add Left (A) or Right (D)
      -> Smoothly transitions into Reverse-Left or Reverse-Right arc!
    - Tap Forward (W) again while arcing -> Straightens back out forward!

  Single Directional Controls:
    Forward : W  or  Up Arrow    or  I
    Reverse : S  or  Down Arrow  or  ,
    Left    : A  or  Left Arrow  or  J
    Right   : D  or  Right Arrow or  L
    Stop    : Spacebar or K (or release all keys)

  Direct Diagonal Shortcuts:
    U : Forward-Left  (↖)       O : Forward-Right (↗)
    M : Reverse-Left  (↙)       . : Reverse-Right (↘)

  Speed Adjustments:
    E / Q : Increase / Decrease Linear Speed (by 10%)
    R / F : Increase / Decrease Turn Speed   (by 10%)
    Ctrl+C to Exit
==============================================================
"""

FIRST_PRESS_TIMEOUT = 0.55
STREAM_REPEAT_TIMEOUT = 0.18


class TeleopNode(Node):
    def __init__(self):
        super().__init__('amr_teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        # Independent Dual-Axis State:
        # longitudinal: +1 (Forward), -1 (Reverse), 0 (Neutral)
        # lateral:      +1 (Turn Left), -1 (Turn Right), 0 (Straight)
        self.longitudinal = 0
        self.lateral = 0

        self.last_key_time = 0.0
        self.is_repeating = False

        self.last_published_linear = 0.0
        self.last_published_angular = 0.0


def drain_all_keys(fd):
    """
    Reads and flushes ALL pending bytes from non-blocking stdin.
    Eliminates buffer backlog and provides sub-millisecond key detection.
    """
    try:
        raw = os.read(fd, 1024)
    except BlockingIOError:
        return []

    if not raw:
        return []

    keys = []
    i = 0
    while i < len(raw):
        if raw[i:i+3] == b'\x1b[A':
            keys.append('UP'); i += 3
        elif raw[i:i+3] == b'\x1b[B':
            keys.append('DOWN'); i += 3
        elif raw[i:i+3] == b'\x1b[C':
            keys.append('RIGHT'); i += 3
        elif raw[i:i+3] == b'\x1b[D':
            keys.append('LEFT'); i += 3
        elif raw[i:i+1] == b'\x1b':
            i += 1
        else:
            try:
                keys.append(chr(raw[i]).lower())
            except ValueError:
                pass
            i += 1

    return keys


def main():
    rclpy.init()
    node = TeleopNode()

    fd = sys.stdin.fileno()
    orig_termios = termios.tcgetattr(fd)
    orig_fl = fcntl.fcntl(fd, fcntl.F_GETFL)

    # Set terminal to raw/cbreak and non-blocking I/O
    tty.setcbreak(fd)
    fcntl.fcntl(fd, fcntl.F_SETFL, orig_fl | os.O_NONBLOCK)

    print(BANNER)
    print(f"Current Speeds -> Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s\n")

    try:
        while rclpy.ok():
            now = time.time()
            keys = drain_all_keys(fd)

            for key in keys:
                if key == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt

                # Track repeating stream timing
                if 0.0 < (now - node.last_key_time) < 0.15:
                    node.is_repeating = True
                else:
                    node.is_repeating = False

                node.last_key_time = now

                # Emergency / Full Stop
                if key in (' ', 'k'):
                    node.longitudinal = 0
                    node.lateral = 0
                    node.is_repeating = False

                # Longitudinal: Forward
                elif key in ('w', 'i', 'UP'):
                    # If already moving forward and user re-presses W, straighten out lateral steering
                    if node.longitudinal == 1 and node.lateral != 0:
                        node.lateral = 0
                    else:
                        node.longitudinal = 1

                # Longitudinal: Backward
                elif key in ('s', ',', 'DOWN'):
                    if node.longitudinal == -1 and node.lateral != 0:
                        node.lateral = 0
                    else:
                        node.longitudinal = -1

                # Lateral: Left
                elif key in ('a', 'j', 'LEFT'):
                    node.lateral = 1

                # Lateral: Right
                elif key in ('d', 'l', 'RIGHT'):
                    node.lateral = -1

                # Direct Diagonal Shortcuts
                elif key == 'u':  # Forward-Left
                    node.longitudinal = 1
                    node.lateral = 1
                elif key == 'o':  # Forward-Right
                    node.longitudinal = 1
                    node.lateral = -1
                elif key == 'm':  # Reverse-Left
                    node.longitudinal = -1
                    node.lateral = 1
                elif key == '.':  # Reverse-Right
                    node.longitudinal = -1
                    node.lateral = -1

                # Speed Tuning
                elif key == 'e':
                    node.linear_speed = min(2.0, node.linear_speed * 1.1)
                    print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="", flush=True)
                elif key == 'q':
                    node.linear_speed = max(0.1, node.linear_speed * 0.9)
                    print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="", flush=True)
                elif key == 'r':
                    node.angular_speed = min(3.0, node.angular_speed * 1.1)
                    print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="", flush=True)
                elif key == 'f':
                    node.angular_speed = max(0.2, node.angular_speed * 0.9)
                    print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="", flush=True)

            # Watchdog: If no character is received within the release timeout, zero both axes
            timeout = STREAM_REPEAT_TIMEOUT if node.is_repeating else FIRST_PRESS_TIMEOUT
            if (now - node.last_key_time) > timeout:
                node.longitudinal = 0
                node.lateral = 0
                node.is_repeating = False

            # Compute velocities from dual-axis state
            target_linear = float(node.longitudinal * node.linear_speed)
            target_angular = float(node.lateral * node.angular_speed)

            # Publish Twist message at 50Hz
            twist = Twist()
            twist.linear.x = target_linear
            twist.angular.z = target_angular
            node.pub.publish(twist)

            # Update live HUD on state change
            if (target_linear != node.last_published_linear or target_angular != node.last_published_angular):
                status_str = "STOPPED ■"
                if target_linear > 0 and target_angular > 0:
                    status_str = "FORWARD-LEFT ↖"
                elif target_linear > 0 and target_angular < 0:
                    status_str = "FORWARD-RIGHT ↗"
                elif target_linear < 0 and target_angular > 0:
                    status_str = "REVERSE-LEFT ↙"
                elif target_linear < 0 and target_angular < 0:
                    status_str = "REVERSE-RIGHT ↘"
                elif target_linear > 0:
                    status_str = "FORWARD ↑"
                elif target_linear < 0:
                    status_str = "REVERSE ↓"
                elif target_angular > 0:
                    status_str = "SPIN-LEFT ↺"
                elif target_angular < 0:
                    status_str = "SPIN-RIGHT ↻"

                print(f"\r[Status] {status_str:<18} (vx={target_linear:+.2f} m/s, wz={target_angular:+.2f} rad/s)    ", end="", flush=True)

            node.last_published_linear = target_linear
            node.last_published_angular = target_angular

            time.sleep(0.02)  # 50 Hz loop

    except (KeyboardInterrupt, Exception):
        pass
    finally:
        # Publish zero velocity on exit to stop robot safely
        try:
            stop_twist = Twist()
            node.pub.publish(stop_twist)
        except Exception:
            pass
        # Restore terminal flags & settings
        fcntl.fcntl(fd, fcntl.F_SETFL, orig_fl)
        termios.tcsetattr(fd, termios.TCSADRAIN, orig_termios)
        print("\nRestored terminal and cleanly stopped teleop node.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
