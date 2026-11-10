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
         AMR High-Performance Real-Time Teleop Node
==============================================================
Drive Controls (Press & Hold, Multi-Key Combinations Supported):
--------------------------------------------------------------
  Multi-Key Diagonal Arcs:
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

# Low-latency key timing constants:
# - STREAM_WINDOW (0.09s): When a key is streaming repeat characters (~30Hz),
#   it expires within 90ms of release for near-instant release detection.
# - INITIAL_WINDOW (0.50s): Bridges the OS keyboard typamatic pause on the first tap.
STREAM_WINDOW = 0.09
INITIAL_WINDOW = 0.50


class TeleopNode(Node):
    def __init__(self):
        super().__init__('amr_teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        # Direction tracking: last press timestamp & repeat count
        self.press_time = {'forward': 0.0, 'backward': 0.0, 'left': 0.0, 'right': 0.0}
        self.press_count = {'forward': 0, 'backward': 0, 'left': 0, 'right': 0}

        self.last_published_linear = 0.0
        self.last_published_angular = 0.0


def drain_all_keys(fd):
    """
    Reads and drains ALL pending bytes from the non-blocking stdin buffer in a single call.
    Completely eliminates character queue backlog and input lag.
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
        # Parse ANSI escape sequences for Arrow keys
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

                # Immediate Stop
                if key in (' ', 'k'):
                    for d in node.press_time:
                        node.press_time[d] = 0.0
                        node.press_count[d] = 0

                # Forward (Cancels opposite direction immediately)
                elif key in ('w', 'i', 'UP'):
                    node.press_count['forward'] = node.press_count['forward'] + 1 if (now - node.press_time['forward'] < 0.2) else 1
                    node.press_time['forward'] = now
                    node.press_time['backward'] = 0.0
                    node.press_count['backward'] = 0

                # Backward (Cancels opposite direction immediately)
                elif key in ('s', ',', 'DOWN'):
                    node.press_count['backward'] = node.press_count['backward'] + 1 if (now - node.press_time['backward'] < 0.2) else 1
                    node.press_time['backward'] = now
                    node.press_time['forward'] = 0.0
                    node.press_count['forward'] = 0

                # Left (Cancels opposite direction immediately)
                elif key in ('a', 'j', 'LEFT'):
                    node.press_count['left'] = node.press_count['left'] + 1 if (now - node.press_time['left'] < 0.2) else 1
                    node.press_time['left'] = now
                    node.press_time['right'] = 0.0
                    node.press_count['right'] = 0

                # Right (Cancels opposite direction immediately)
                elif key in ('d', 'l', 'RIGHT'):
                    node.press_count['right'] = node.press_count['right'] + 1 if (now - node.press_time['right'] < 0.2) else 1
                    node.press_time['right'] = now
                    node.press_time['left'] = 0.0
                    node.press_count['left'] = 0

                # Legacy Diagonal Shortcuts
                elif key == 'u':  # Forward-Left
                    node.press_time['forward'] = now; node.press_count['forward'] = 2
                    node.press_time['left'] = now;    node.press_count['left'] = 2
                    node.press_time['backward'] = 0.0; node.press_time['right'] = 0.0
                elif key == 'o':  # Forward-Right
                    node.press_time['forward'] = now; node.press_count['forward'] = 2
                    node.press_time['right'] = now;   node.press_count['right'] = 2
                    node.press_time['backward'] = 0.0; node.press_time['left'] = 0.0
                elif key == 'm':  # Reverse-Left
                    node.press_time['backward'] = now; node.press_count['backward'] = 2
                    node.press_time['left'] = now;     node.press_count['left'] = 2
                    node.press_time['forward'] = 0.0;  node.press_time['right'] = 0.0
                elif key == '.':  # Reverse-Right
                    node.press_time['backward'] = now; node.press_count['backward'] = 2
                    node.press_time['right'] = now;    node.press_count['right'] = 2
                    node.press_time['forward'] = 0.0;  node.press_time['left'] = 0.0

                # Speed Adjustments
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

            # Evaluate active directions:
            # - If repeating (>1 press in succession), window is 90ms for instant release.
            # - If initial press, window is 500ms to absorb OS typamatic gap.
            def is_active(direction):
                window = STREAM_WINDOW if node.press_count[direction] > 1 else INITIAL_WINDOW
                return (now - node.press_time[direction]) < window

            fwd_active = is_active('forward')
            bwd_active = is_active('backward')
            left_active = is_active('left')
            right_active = is_active('right')

            # Calculate resultant commanded velocities
            target_linear = 0.0
            if fwd_active and not bwd_active:
                target_linear = node.linear_speed
            elif bwd_active and not fwd_active:
                target_linear = -node.linear_speed

            target_angular = 0.0
            if left_active and not right_active:
                target_angular = node.angular_speed
            elif right_active and not left_active:
                target_angular = -node.angular_speed

            # Publish Twist message at 50Hz
            twist = Twist()
            twist.linear.x = float(target_linear)
            twist.angular.z = float(target_angular)
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
