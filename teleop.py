#!/usr/bin/env python3
import sys
import os
import time
import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class TeleopNode(Node):
    def __init__(self, odom_callback=None):
        super().__init__('amr_teleop_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_callback = odom_callback
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._handle_odom,
            10
        )

    def _handle_odom(self, msg):
        if self.odom_callback:
            self.odom_callback(msg)


class AMRTeleopApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node

        self.root.title("AMR Drive Control Pad & Telemetry")
        self.root.geometry("440x490")
        self.root.configure(bg="#1E1E2E")
        self.root.resizable(False, False)

        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

        # Active physical keys tracked via true KeyPress / KeyRelease
        self.pressed_keys = set()
        self.release_timers = {}

        self.last_published_linear = 0.0
        self.last_published_angular = 0.0

        # Live Odometry Telemetry
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw_deg = 0.0
        self.current_speed = 0.0
        self.total_distance = 0.0
        self.last_odom_x = None
        self.last_odom_y = None

        self._build_ui()
        self._bind_keys()

        # Start 50 Hz control loop (every 20 ms)
        self.root.after(20, self._control_loop)

    def on_odom_received(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        w = msg.pose.pose.orientation.w
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z

        yaw_rad = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.yaw_deg = math.degrees(yaw_rad)

        if self.last_odom_x is not None:
            step_dist = math.hypot(x - self.last_odom_x, y - self.last_odom_y)
            # Only accumulate realistic motion (> 1mm)
            if step_dist > 0.001:
                self.total_distance += step_dist

        self.last_odom_x = x
        self.last_odom_y = y
        self.pos_x = x
        self.pos_y = y

        vx = msg.twist.twist.linear.x
        self.current_speed = abs(vx)

    def _normalize_key(self, event):
        k = event.char.lower() if event.char else ''
        sym = event.keysym
        if sym in ('Up', 'w', 'W'): return 'up'
        if sym in ('Down', 's', 'S'): return 'down'
        if sym in ('Left', 'a', 'A'): return 'left'
        if sym in ('Right', 'd', 'D'): return 'right'
        if sym in ('i', 'I'): return 'up'
        if sym in (',',): return 'down'
        if sym in ('j', 'J'): return 'left'
        if sym in ('l', 'L'): return 'right'
        if k in ('u', 'o', 'm', '.', ' ', 'k', 'e', 'q', 'r', 'f'):
            return k
        return None

    def _on_key_press(self, event):
        key = self._normalize_key(event)
        if not key:
            return

        # Cancel any pending release timer (X11 auto-repeat bounce filter)
        if key in self.release_timers:
            self.root.after_cancel(self.release_timers.pop(key))

        if key == ' ':
            self.pressed_keys.clear()
        elif key == 'e':
            self.linear_speed = min(2.0, self.linear_speed * 1.1)
            self._update_speed_label()
        elif key == 'q':
            self.linear_speed = max(0.1, self.linear_speed * 0.9)
            self._update_speed_label()
        elif key == 'r':
            self.angular_speed = min(3.0, self.angular_speed * 1.1)
            self._update_speed_label()
        elif key == 'f':
            self.angular_speed = max(0.2, self.angular_speed * 0.9)
            self._update_speed_label()
        else:
            self.pressed_keys.add(key)

    def _on_key_release(self, event):
        key = self._normalize_key(event)
        if not key:
            return

        # Schedule release after 35ms to ignore synthetic X11 repeat bounces
        if key in self.release_timers:
            self.root.after_cancel(self.release_timers.pop(key))
        self.release_timers[key] = self.root.after(35, self._finalize_release, key)

    def _finalize_release(self, key):
        self.release_timers.pop(key, None)
        self.pressed_keys.discard(key)

    def _compute_velocities(self):
        fwd = bool(self.pressed_keys & {'up', 'u', 'o'})
        bwd = bool(self.pressed_keys & {'down', 'm', '.'})
        left = bool(self.pressed_keys & {'left', 'u', 'm'})
        right = bool(self.pressed_keys & {'right', 'o', '.'})

        vx = 0.0
        if fwd and not bwd:
            vx = self.linear_speed
        elif bwd and not fwd:
            vx = -self.linear_speed

        wz = 0.0
        if left and not right:
            wz = self.angular_speed
        elif right and not left:
            wz = -self.angular_speed

        return vx, wz

    def _control_loop(self):
        vx, wz = self._compute_velocities()

        # Publish Twist command at 50Hz
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)
        self.node.pub.publish(twist)

        # Update UI status if changed
        if vx != self.last_published_linear or wz != self.last_published_angular:
            self._update_status_display(vx, wz)
            self.last_published_linear = vx
            self.last_published_angular = wz

        # Update Live Odometry Telemetry in HUD
        self._update_odom_display()

        # Schedule next tick
        self.root.after(20, self._control_loop)

    def _update_status_display(self, vx, wz):
        # Reset button colors
        for btn in self.dir_buttons.values():
            btn.configure(bg="#313244", fg="#CDD6F4")

        status_text = "STOPPED ■"
        active_key = None

        if vx > 0 and wz > 0:
            status_text = "FORWARD-LEFT ↖"
            active_key = 'fl'
        elif vx > 0 and wz < 0:
            status_text = "FORWARD-RIGHT ↗"
            active_key = 'fr'
        elif vx < 0 and wz > 0:
            status_text = "REVERSE-LEFT ↙"
            active_key = 'rl'
        elif vx < 0 and wz < 0:
            status_text = "REVERSE-RIGHT ↘"
            active_key = 'rr'
        elif vx > 0:
            status_text = "FORWARD ↑"
            active_key = 'f'
        elif vx < 0:
            status_text = "REVERSE ↓"
            active_key = 'b'
        elif wz > 0:
            status_text = "SPIN LEFT ↺"
            active_key = 'l'
        elif wz < 0:
            status_text = "SPIN RIGHT ↻"
            active_key = 'r'
        else:
            active_key = 'c'

        if active_key and active_key in self.dir_buttons:
            self.dir_buttons[active_key].configure(bg="#89B4FA", fg="#11111B")

        self.status_label.configure(
            text=f"Status: {status_text} (vx={vx:+.2f} m/s, wz={wz:+.2f} rad/s)"
        )

    def _update_odom_display(self):
        self.odom_label.configure(
            text=f"📍 Pose: X: {self.pos_x:+.2f} m | Y: {self.pos_y:+.2f} m | Yaw: {self.yaw_deg:+.1f}°\n"
                 f"⚡ Speed: {self.current_speed:.2f} m/s | 🏁 Odometer: {self.total_distance:.2f} m"
        )

    def _update_speed_label(self):
        self.speed_label.configure(
            text=f"Linear Target: {self.linear_speed:.2f} m/s (E/Q) | Turn Target: {self.angular_speed:.2f} rad/s (R/F)"
        )

    def _bind_keys(self):
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        try:
            stop_twist = Twist()
            self.node.pub.publish(stop_twist)
        except Exception:
            pass
        self.root.destroy()

    def _build_ui(self):
        # Title Header
        title = tk.Label(
            self.root,
            text="AMR Drive Teleoperation & HUD",
            font=("Helvetica", 14, "bold"),
            bg="#1E1E2E",
            fg="#89B4FA"
        )
        title.pack(pady=(10, 2))

        subtitle = tk.Label(
            self.root,
            text="Keep focused. Multi-key combinations (W/A/S/D & Arrows) supported:",
            font=("Helvetica", 9),
            bg="#1E1E2E",
            fg="#A6ADC8"
        )
        subtitle.pack(pady=(0, 8))

        # 3x3 Directional Pad
        pad_frame = tk.Frame(self.root, bg="#1E1E2E")
        pad_frame.pack(pady=2)

        btn_font = ("Helvetica", 13, "bold")
        btn_w, btn_h = 5, 2

        self.dir_buttons = {}

        layout = [
            [('fl', '↖'), ('f', '↑'),  ('fr', '↗')],
            [('l',  '↺'), ('c', '■'),  ('r',  '↻')],
            [('rl', '↙'), ('b', '↓'),  ('rr', '↘')]
        ]

        for r, row in enumerate(layout):
            for c, (ident, symbol) in enumerate(row):
                b = tk.Label(
                    pad_frame,
                    text=symbol,
                    font=btn_font,
                    width=btn_w,
                    height=btn_h,
                    bg="#313244",
                    fg="#CDD6F4",
                    relief="raised",
                    bd=1
                )
                b.grid(row=r, column=c, padx=3, pady=3)
                self.dir_buttons[ident] = b

        # Status Label
        self.status_label = tk.Label(
            self.root,
            text="Status: STOPPED ■ (vx=+0.00 m/s, wz=+0.00 rad/s)",
            font=("Courier", 10, "bold"),
            bg="#1E1E2E",
            fg="#A6E3A1",
            justify="center"
        )
        self.status_label.pack(pady=(8, 4))

        # Live Odometry Telemetry Card
        odom_card = tk.LabelFrame(
            self.root,
            text=" Live Odometry Telemetry (/odom) ",
            font=("Helvetica", 9, "bold"),
            bg="#181825",
            fg="#CBA6F7",
            padx=10,
            pady=6,
            relief="groove"
        )
        odom_card.pack(fill="x", padx=16, pady=6)

        self.odom_label = tk.Label(
            odom_card,
            text="📍 Pose: X: +0.00 m | Y: +0.00 m | Yaw: +0.0°\n⚡ Speed: 0.00 m/s | 🏁 Odometer: 0.00 m",
            font=("Courier", 10),
            bg="#181825",
            fg="#F5C2E7",
            justify="center"
        )
        self.odom_label.pack()

        # Speed Setting Info
        self.speed_label = tk.Label(
            self.root,
            text=f"Linear Target: {self.linear_speed:.2f} m/s (E/Q) | Turn Target: {self.angular_speed:.2f} rad/s (R/F)",
            font=("Helvetica", 9),
            bg="#1E1E2E",
            fg="#BAC2DE",
            justify="center"
        )
        self.speed_label.pack(pady=(4, 6))


def main():
    rclpy.init()

    # Pre-declare node and app references
    app_ref = None

    def odom_bridge(msg):
        if app_ref:
            app_ref.on_odom_received(msg)

    node = TeleopNode(odom_callback=odom_bridge)

    # Spin ROS 2 in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("==============================================================")
    print("Launching AMR Drive Control Pad & Telemetry HUD...")
    print("Features:")
    print("  - Drive Controls: W / A / S / D  or  Arrow Keys")
    print("  - Dynamic Chording: Hold W, tap/hold A/D to steer dynamically")
    print("  - Live Odometry Display: Coordinates, Heading & Odometer")
    print("  - Speed Tuning: E/Q (Linear speed), R/F (Turn speed)")
    print("  - Stop : Spacebar or release all keys")
    print("==============================================================")

    root = tk.Tk()
    app = AMRTeleopApp(root, node)
    app_ref = app

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_twist = Twist()
            node.pub.publish(stop_twist)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        print("Teleop closed cleanly.")


if __name__ == '__main__':
    main()
