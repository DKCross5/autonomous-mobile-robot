#!/usr/bin/env python3
import sys
import select
import termios
import tty
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
==============================================================
            AMR Interactive Keyboard Teleop Node
==============================================================
Drive Controls (Press & Hold, Multi-Key Combinations Supported):
--------------------------------------------------------------
  Combined Chording:
    Forward + Left   (W+A or Up+Left or U)    : Arc Forward-Left  (↖)
    Forward + Right  (W+D or Up+Right or O)   : Arc Forward-Right (↗)
    Reverse + Left   (S+A or Down+Left or M)  : Arc Reverse-Left  (↙)
    Reverse + Right  (S+D or Down+Right or .) : Arc Reverse-Right (↘)

  Single Directions:
    Forward : W  or  Up Arrow    or  I
    Reverse : S  or  Down Arrow  or  ,
    Left    : A  or  Left Arrow  or  J
    Right   : D  or  Right Arrow or  L
    Stop    : Spacebar or K

  Speed Adjustments:
    E / Q : Increase / Decrease Linear Speed (by 10%)
    R / F : Increase / Decrease Turn Speed   (by 10%)
    Ctrl+C to Exit
==============================================================
"""

# Key persistence window (seconds): if a key event was seen within this window,
# it is considered actively held down by the user.
KEY_HOLD_WINDOW = 0.20


class TeleopNode(Node):
    def __init__(self):
        super().__init__('amr_teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        # Key activity timestamps
        self.key_press_time = {
            'forward': 0.0,
            'backward': 0.0,
            'left': 0.0,
            'right': 0.0
        }

        self.last_published_linear = 0.0
        self.last_published_angular = 0.0


def read_key(settings, timeout=0.03):
    """Non-blocking single character or escape sequence reader."""
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return None

    ch = sys.stdin.read(1)
    # Check for escape sequences (e.g. arrow keys: \x1b[A, \x1b[B, etc.)
    if ch == '\x1b':
        # Check if more characters follow in the escape sequence
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        if rlist:
            seq = sys.stdin.read(2)
            if seq == '[A':
                return 'UP'
            elif seq == '[B':
                return 'DOWN'
            elif seq == '[C':
                return 'RIGHT'
            elif seq == '[D':
                return 'LEFT'
        return 'ESC'
    return ch.lower()


def main():
    rclpy.init()
    node = TeleopNode()

    # Save original terminal settings
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    print(BANNER)
    print(f"Current Speeds -> Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s\n")

    try:
        while rclpy.ok():
            key = read_key(orig_settings, timeout=0.03)
            now = time.time()

            if key == '\x03':  # Ctrl+C
                break

            # Immediate Emergency Stop
            if key in (' ', 'k'):
                for direction in node.key_press_time:
                    node.key_press_time[direction] = 0.0

            # Forward Keys
            elif key in ('w', 'i', 'UP'):
                node.key_press_time['forward'] = now
            # Backward Keys
            elif key in ('s', ',', 'DOWN'):
                node.key_press_time['backward'] = now
            # Left Keys
            elif key in ('a', 'j', 'LEFT'):
                node.key_press_time['left'] = now
            # Right Keys
            elif key in ('d', 'l', 'RIGHT'):
                node.key_press_time['right'] = now

            # Legacy Diagonal Shortcuts (U, O, M, .)
            elif key == 'u':
                node.key_press_time['forward'] = now
                node.key_press_time['left'] = now
            elif key == 'o':
                node.key_press_time['forward'] = now
                node.key_press_time['right'] = now
            elif key == 'm':
                node.key_press_time['backward'] = now
                node.key_press_time['left'] = now
            elif key == '.':
                node.key_press_time['backward'] = now
                node.key_press_time['right'] = now

            # Speed Tuning
            elif key == 'e':
                node.linear_speed = min(2.0, node.linear_speed * 1.1)
                print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="")
            elif key == 'q':
                node.linear_speed = max(0.1, node.linear_speed * 0.9)
                print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="")
            elif key == 'r':
                node.angular_speed = min(3.0, node.angular_speed * 1.1)
                print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="")
            elif key == 'f':
                node.angular_speed = max(0.2, node.angular_speed * 0.9)
                print(f"\r[Speed] Linear: {node.linear_speed:.2f} m/s | Angular: {node.angular_speed:.2f} rad/s    ", end="")

            # Evaluate active held directions based on recent keypress events
            fwd_active = (now - node.key_press_time['forward']) < KEY_HOLD_WINDOW
            bwd_active = (now - node.key_press_time['backward']) < KEY_HOLD_WINDOW
            left_active = (now - node.key_press_time['left']) < KEY_HOLD_WINDOW
            right_active = (now - node.key_press_time['right']) < KEY_HOLD_WINDOW

            # Calculate resultant commanded linear velocity
            target_linear = 0.0
            if fwd_active and not bwd_active:
                target_linear = node.linear_speed
            elif bwd_active and not fwd_active:
                target_linear = -node.linear_speed

            # Calculate resultant commanded angular velocity
            target_angular = 0.0
            if left_active and not right_active:
                target_angular = node.angular_speed
            elif right_active and not left_active:
                target_angular = -node.angular_speed

            # Publish Twist message at ~30Hz
            twist = Twist()
            twist.linear.x = float(target_linear)
            twist.angular.z = float(target_angular)
            node.pub.publish(twist)

            # Informative live status line when state changes
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

            time.sleep(0.033)

    except Exception as e:
        print(f"\nError in teleop: {e}")
    finally:
        # Stop robot before exiting
        stop_twist = Twist()
        node.pub.publish(stop_twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("\nRestored terminal and cleanly stopped teleop node.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
