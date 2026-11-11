# AMR Digital Twin Project: Master Technical Briefing & Phase Handoff

> [!IMPORTANT]
> **🤖 AI Agent Ingestion Directive:**
> If you are an AI assistant starting a new conversation session, read this document to immediately ingest the complete technical architecture, calibrated kinematics, and ground rules. Once reviewed, confirm ingestion to the user and immediately proceed with executing **Section 6: Immediate Next Step (Phase 3 Gate)**. Always follow **Section 7: Rules of Engagement, Engineering Philosophy & Assistant Directives** (including the Living Document Protocol to continuously record and update this file with any new methodologies, operator preferences, or directives discussed with the user).

## 1. Executive Summary & Core Objective
- **Mission:** Transform a manual RC car chassis into a fully autonomous mobile robot (AMR) capable of 2D/3D SLAM mapping, obstacle avoidance, and Nav2 waypoint navigation in simulation (MuJoCo digital twin) before deploying to physical hardware.
- **Physical Hardware Target:**
  - **Chassis:** 3-wheel differential drive (two 6-inch rubber drive wheels, 1 rear caster).
  - **Motor Driver:** Sabertooth 2x25 in Mixed Mode (single-joystick differential mixing).
  - **Power:** 24V (series-wired LiFePO4 battery pack).
  - **Sensors:** ROBOTIS LDS-01 360° LiDAR, Kinect depth camera, USB webcams.
  - **RC Rig:** Spektrum DXe transmitter + OrangeRx receiver.
- **Environment:** Ubuntu 22.04 LTS via WSL2 (WSLg graphical display), ROS 2 Humble, MuJoCo 3.x, Python 3.10.

---

## 2. Active Codebase Architecture & File Registry

| File | Type | Role & Capabilities |
| :--- | :--- | :--- |
| `my_amr.xml` | MuJoCo MJCF | Physics model: Differential drive chassis, 6-inch rubber wheels ($R=0.0762\text{m}$, track $L=0.36\text{m}$), motor actuators (`armature="0.01"`, `kv="3.0"`), low-friction caster contact pair (`friction="0.001"`), red obstacle wall ($X=2.5\text{m}$), spinning LiDAR joint with rangefinder sensor. |
| `sim_test.py` | ROS 2 / MuJoCo | Core simulation node (500Hz physics, 50Hz throttled viewer sync). Background threaded ROS executor (`rclpy.spin`). Dynamic slew rate filters, typamatic deadman watchdog (`FIRST_PRESS=0.55s`, `HOLD=0.15s`). Dynamic TF2 broadcaster (`odom -> base_link -> lidar_link`), publishes `/odom`, `/robot_path`, `/robot_visual` (MarkerArray), `/scan` (full 360-beam `LaserScan`). Subscribes to `/cmd_vel`. |
| `teleop.py` | Tkinter / ROS 2 | Dark-theme GUI Control Pad and Telemetry HUD. Runs in WSLg. True `<KeyPress>` / `<KeyRelease>` tracking with 35ms X11 debounce filter. 50Hz publish rate. Full multi-key chord support (drive + steer simultaneously). Live telemetry card ($X, Y, \text{Yaw}, \text{Speed}, \text{Odometer}$). |
| `amr_view.rviz` | YAML Config | Pre-configured RViz2 layout: Fixed Frame `odom`, dark slate background (`#1E1E2E`), Grid, TF tree, 3D AMR MarkerArray, neon-green Path trail, cyan 360-beam LaserScan point cloud, orange Odometry pose arrow, calibrated Orbit camera. |
| `launch_rviz.sh` | Shell Executable | Automated RViz2 launcher: auto-sources ROS 2 Humble, loads `amr_view.rviz`, displays formatted status banner. |
| `start_amr.sh` | Shell Executable | Master one-command launcher: starts sim, rviz, and teleop with clean signal trapping and child PID teardown. |
| `start_interactive.sh` | Shell Executable | Multi-pane `tmux` dashboard: splits terminal into 3 live interactive bash shells with full mouse support. |
| `.vscode/tasks.json` | VS Code Tasks | Tasks for sim, teleop, rviz, and parallel launch. All hardened with `; exec bash` interactive fallback. |
| `.vscode/keybindings_reference.json` | Keybinding Template | Keybindings (`Alt+Shift+T` for 3 tabs, `Alt+Shift+S` for 3 side-by-side splits, `Alt+T` for clean terminals). |

---

## 3. Calibrated Technical Parameters

```python
# Kinematics & Physics
WHEEL_RADIUS      = 0.0762   # meters (6-inch diameter drive wheels)
WHEEL_BASE        = 0.36     # meters (distance between left & right wheel centerlines)
PHYSICS_STEP_DT   = 0.002    # seconds (500 Hz simulation rate)
VIEWER_SYNC_RATE  = 100      # Hz (synced to 100 FPS for ultra-fluid rendering)

# Actuator & Contact Dynamics
MOTOR_KV          = 3.0      # Velocity feedback damping gain
ROTOR_ARMATURE    = 0.01     # Wheel rotor inertia (prevents QACC explosion)
CASTER_FRICTION   = 0.001    # Eliminates yaw drift caused by default floor friction

# Watchdog & Slew Rate Filtering
FIRST_PRESS_TO    = 0.55     # seconds (absorbs OS typamatic repeat delay)
HOLD_TIMEOUT      = 0.15     # seconds (deadman safety stop)
STEER_LIN_ACCEL   = 4.0      # m/s^2 (active steering acceleration)
STEER_ANG_ACCEL   = 10.0     # rad/s^2
BRAKE_LIN_DECEL   = 2.0      # m/s^2 (smooth coasting deceleration)
BRAKE_ANG_DECEL   = 5.0      # rad/s^2

# LaserScan Synthesis (ROBOTIS LDS-01 spec emulation via mujoco.mj_ray)
LIDAR_NUM_BEAMS      = 360   # 1° angular resolution (REP-117 compliant)
LIDAR_RANGE_MIN      = 0.12  # meters (LDS-01 blind-spot cutoff)
LIDAR_RANGE_MAX      = 3.50  # meters (LDS-01 max reliable range)
LIDAR_SCAN_HZ        = 20.0  # Hz  (one scan every 25 physics steps @ 500 Hz)
LIDAR_TIME_INC       = 0.0   # seconds (instantaneous simulation scan; eliminates rolling-shutter seams)
LIDAR_MOUNT_Z_NUDGE  = 0.025 # meters (ray-origin upward nudge clears chassis deck)
```

---

## 4. Solved Architectural Pitfalls & Lessons Learned

1. **WSL OpenGL Hardware Acceleration Glitch:**
   - *Fix:* `export LIBGL_ALWAYS_SOFTWARE=1` in `~/.bashrc` ensures stable software rasterization and prevents black/frozen screens in MuJoCo and RViz2.
2. **QACC DOF 6 Physics Explosion (`Nan/Inf` acceleration):**
   - *Cause:* High velocity gain on low-inertia wheel joints without rotor armature.
   - *Fix:* Added `armature="0.01"` and set `kv="3.0"` in `my_amr.xml`.
3. **Caster Floor Friction Yaw Drift:**
   - *Cause:* MuJoCo contact friction defaults to maximum of geom frictions. The floor friction (1.0) caused the rear caster to drag with ~70N of lateral friction.
   - *Fix:* Added explicit contact pair `<pair geom1="floor" geom2="caster_ball" friction="0.001 0.001 ..."/>` in `my_amr.xml`.
4. **Teleop Multi-Key Chording & Key Drop-out:**
   - *Cause:* Terminal stdin only receives character streams and OS keyboard repeat pre-empts held keys.
   - *Fix:* Built a Tkinter GUI tracking true X11 KeyPress/KeyRelease events with a 35ms debounce filter in `teleop.py`.
5. **Interactive Terminals vs. Task Runners:**
   - *Lesson:* Task runners lock terminals on exit. Interactive terminals with persistent bash prompts (or `; exec bash`) are essential for rapid robotics iteration and `Up-Arrow` re-runs.
6. **`mj_ray` Self-Hit (LiDAR Zero-Distance Returns):**
   - *Cause:* `mujoco.mj_ray()` with `bodyexclude=-1` (no exclusion) fires from inside or on the surface of the LiDAR body geom, returning `dist=0.0` self-intersections for all beams.
   - *Fix:* Cache `lidar_body_id = mj_name2id(model, mjOBJ_BODY, "lidar")` at startup. Pass it as `bodyexclude` to every `mj_ray` call, and offset the ray origin `+0.025 m` in Z from `data.xpos[lidar_body_id]` to clear the chassis deck.
   - *Validated:* 47 correct wall-hits at 2.35 m, zero self-hits across all 360 beams.
7. **Stationary vs. Rotating TF for Omnidirectional LaserScans:**
   - *Cause:* In Phase 2, `lidar_link` had a dynamic TF rotation matching `qpos[9]` because the rangefinder was a single ray on a spinning motor. In Phase 3, we synthesized a full 360-beam planar `LaserScan` where beam angles are already indexed relative to the robot. Because `lidar_link` continued rotating in TF at 10 rad/s, RViz rotated the entire 360-beam array by `lidar_angle`, causing the static wall reflection to spin in a circle around the AMR tangent to its orbit.
   - *Fix:* Set `lidar_link` rotation in `publish_transforms` to identity (fixed mount: `w=1.0, x=0, y=0, z=0`). The MuJoCo physical green cylinder can still visually spin, but the ROS TF frame remains rigidly mounted to `base_link` per REP-105.
   - *Validated:* Laser returns stay locked rock-solid on the obstacle wall without rotating.
8. **WSL2 / Linux Timer Resolution & Simulation Stutter:**
   - *Cause:* Calling `time.sleep()` on every 0.002s physics step (500 times per second) suffers from OS scheduler quantization (~1.5–3.0ms per sleep), dragging physics down to ~350–470 Hz and causing visual micro-stuttering in both MuJoCo and RViz2.
   - *Fix:* Batch real-time pacing to display sync intervals (every 5 physics steps = 100 Hz) using a drift-free wall-clock time accumulator. Bump RViz2 render limit to 120 FPS (`Frame Rate: 120`), and broadcast TF, Odometry, and `robot_visual` MarkerArray at 100 Hz.
   - *Validated:* Zero timer drift, physics runs at solid 500 Hz, RViz renders at 120 FPS.
9. **LaserScan `time_increment` Rolling-Shutter Discontinuity Seam:**
   - *Cause:* Setting `scan.time_increment = (1/Hz) / 360` (~0.00028s) instructed RViz2 to interpolate the robot's TF pose for each individual beam across the 100ms sweep. In MuJoCo simulation, all 360 rays are cast simultaneously at a single instant $t$. Because beam 0 ($0^\circ$) and beam 359 ($359^\circ$) meet directly in front of the robot (+X), any linear or angular motion during the scan caused RViz2 to project beam 359 at a different robot pose than beam 0, creating a visible seam/discontinuity directly in front of where the AMR faces.
   - *Fix:* Set `scan.time_increment = 0.0`. Per ROS REP-117, when all measurements are captured at the same instant (as in simulation), `time_increment` must be 0.0. This eliminates the seam completely.
10. **RViz2 `Use Fixed Frame: false` Point Cloud Swaying & Rotation:**
    - *Cause:* `amr_view.rviz` had `Use Fixed Frame: false` for the LaserScan display. With this setting, RViz2 re-transforms the scan points on every render frame using the newest `lidar_link` TF rather than the timestamp of the scan. When turning or accelerating/decelerating, the points were dragged along with the robot's movement, causing the blue line to sway and float off the wall.
    - *Fix:* Set `Use Fixed Frame: true` in `amr_view.rviz`. RViz2 now projects the points into `odom` at the scan timestamp, anchoring the laser points permanently to the obstacle wall.

---

## 5. Phased Engineering Roadmap

```
Phase 1: Model & Physics Baseline [COMPLETED]
Phase 2: Closed-Loop Teleop & Telemetry Visualization [COMPLETED]
Phase 3: Sensor Realism & 2D LiDAR Scan Emulation [COMPLETED]
Phase 4: 2D SLAM Mapping (Cartographer / slam_toolbox) [ACTIVE / NEXT]
Phase 5: Autonomous Navigation (Nav2 Costmaps & Planners) [PENDING]
Phase 6: Hardware-in-the-Loop & Physical Deployment [PENDING]
```

### Phase Gate Checklist
- [x] **Phase 1 Gate:** MuJoCo 3-wheel model drives without exploding; ROS 2 bridge publishes telemetry and receives `/cmd_vel`.
- [x] **Phase 2 Gate:** Fluid multi-key teleoperation HUD; dynamic TF2 broadcasting (`odom -> base_link -> lidar_link`); persistent RViz2 layout with 3D markers and path.
- [x] **Phase 3 Gate (COMPLETED):**
  - ✅ Full 360-beam `sensor_msgs/msg/LaserScan` on `/scan` synthesized via `mujoco.mj_ray()` at 10 Hz, matching ROBOTIS LDS-01 spec (1° resolution, 0.12–3.5 m range).
  - ✅ Zero self-hits via `bodyexclude=lidar_body_id`; 47 clean wall-hit returns validated at 2.35 m.
  - ✅ `amr_view.rviz` updated: `Range` cone display replaced by `LaserScan` point cloud display (cyan, `Style: Points`, subscribed to `/scan`).
  - ⏳ Noisy wheel encoder odometry (Phase 3 stretch goal) deferred to Phase 4 — `slam_toolbox` can be integrated with ground-truth odometry first.
- [ ] **Phase 4 Gate (CURRENT):** Generate and save a 2D occupancy grid map (`/map`) using `slam_toolbox`.
- [ ] **Phase 5 Gate:** Autonomous navigation to goal poses avoiding dynamic obstacles via Nav2.
- [ ] **Phase 6 Gate:** Deploy ROS 2 control stack to physical Sabertooth 2x25 driver and hardware LDS-01 sensor.

---

## 6. Immediate Next Step (For the New Chat Session)
**Target:** Integrate `slam_toolbox` to generate a live 2D occupancy grid map (`/map`) from the `/scan` LaserScan topic.
- **Why:** With a standards-compliant 360-beam `/scan` now publishing, the pipeline is ready for SLAM. `slam_toolbox` in `online_async` mode will consume `/scan` + `/tf` (`odom -> base_link -> lidar_link`) and emit a `/map` occupancy grid that persists across the room as the robot drives.
- **Approach:**
  1. Install `slam_toolbox` Humble package (`sudo apt install ros-humble-slam-toolbox`).
  2. Create `slam_params.yaml` tuned to our model (5m×5m arena, 10 Hz scan, ground-truth `/odom`).
  3. Add `slam_toolbox` node launch to `start_amr.sh` with clean SIGINT teardown.
  4. Add `/map` Occupancy Grid display to `amr_view.rviz` and verify map builds correctly as robot is teleoperated around the red wall obstacle.
  5. Add a `save_map.sh` helper (wraps `ros2 run nav2_map_server map_saver_cli`) for persisting maps to disk.

---

## 7. Rules of Engagement, Engineering Philosophy & Assistant Directives

> [!IMPORTANT]
> **Living Document Protocol:** If ANY new methodology, design decision, operator preference, or rule of engagement is discussed with the user during a session, the active AI assistant **MUST record and update this document** before concluding the session. `PROJECT_HANDOFF.md` is the living Single Source of Truth (SSOT).

### Core Directives & Standards:
1. **Automatic Git Commits:**
   - After *every* functional code change or milestone, automatically run validation tests and commit the changes via `git commit`.
   - Every commit message MUST be elaborate, clean, and structured (Context & Motivation, Key Technical Breakdown, Verification & Impact).
2. **Living Documentation & Session Handoff Maintenance:**
   - Continuously update this document with newly calibrated physical parameters, resolved bugs, architectural changes, and roadmap progress.
   - When asked for a handoff, update Section 6 with the next exact Phase Gate and provide the user with the concise 1-sentence prompt for the incoming AI.
3. **Efficient Coding & Logical Methodology:**
   - **First-Principles Robotics:** Code from physical fundamentals (wheel radius, track width, motor torque/armature, kinematics). Never use arbitrary fudge factors or magic numbers without clear physical units and commentary.
   - **Modular & Defensive:** Write clean, readable, single-responsibility functions with bounds checking and graceful error recovery. Keep simulations robust against NaN/Inf instabilities and timing jitter.
   - **No Premature Abstractions:** Keep code directly runnable, transparent, and modular. Do not introduce heavy boilerplate, overly complex package structures, or unnecessary dependencies until explicitly required by Nav2/SLAM.
4. **User-Friendliness & Developer Ergonomics:**
   - **Interactive-First Workflow:** Maintain interactive terminal support (`start_interactive.sh`, VS Code keybindings `Alt+Shift+T` and `Alt+Shift+S`, and `; exec bash` fallback in tasks) so shells remain alive for instant `<Up-Arrow>` re-runs and ROS 2 CLI introspection.
   - **One-Click Automation:** Provide turnkey scripts (`start_amr.sh`, `launch_rviz.sh`) with automatic environment detection, ROS 2 sourcing, and clean SIGINT/SIGTERM teardown.
   - **Terminal Polish:** Use formatted ANSI color banners, progress indicators, and informative status logs so the operator always knows system state at a glance.
5. **"Wow-Factor", Showcase Polish & Cutting-Edge Flair:**
   - **For Complete Beginners:** The project must be immediately visual, intuitive, and delightful. High-contrast dark themes, real-time 3D robot models in RViz2, neon trajectory trails, and an accessible GUI control pad allow anyone to pick up the controls and understand what the robot is doing within seconds.
   - **For Industry Veterans:** The project must demonstrate undeniable engineering rigor. Perfect ROS 2 Humble conventions, disciplined TF2 dynamic transform trees (`odom -> base_link -> lidar_link`), accurate differential drive kinematics, realistic contact friction, and zero sloppy hacks.
   - **For Cutting-Edge Innovators:** Add unique, bespoke flair that elevates the project beyond standard textbook tutorials. Examples include our dual dynamic slew rate filters (asymmetric acceleration vs. coasting deceleration), true multi-key chord teleoperation with X11 debounce filtering, live HUD telemetry cards (odometer, digital speedometer, orientation yaw), and sleek visual dashboards. Look for creative opportunities to innovate on telepresence, data visualization, and simulation realism at every step.
6. **Operator Verification, Checkpoint Protocol & Concrete Testing Prompts:**
   - Never conclude a functional code change or milestone by declaring it finished and prompting to push ahead to the next phase without an interactive verification checkpoint.
   - Always provide the operator with:
     a) **How to Launch/Test:** The exact commands, terminal steps, or one-click shortcuts to test the changes.
     b) **What to Expect Visually:** Detailed visual checkpoints in MuJoCo, RViz2, terminal logs, or the Teleop GUI (e.g. colors, shapes, alignment, stability, topic rates).
     c) **Interactive Test Recipe:** Concrete actions for the operator to perform (e.g. 'Press W to drive forward towards the wall, observe that cyan dots remain glued to the red wall face without rotating').

<!-- [For future reference, these are the prompts that will connect any string of future chats together with this PROJECT_HANDOFF file]
The Updated Reusable "Chat Handoff Generator" Meta-Prompt
(Save this. Paste this prompt at the end of future chats when you are ready to migrate again):

We are migrating to a fresh chat session to conserve credits.

Please perform our standard project handoff:

1. Update PROJECT_HANDOFF.md with all progress, newly calibrated parameters, solved pitfalls, and roadmap status achieved in this chat.
2. In Section 7 of PROJECT_HANDOFF.md, record any new design methodologies, operator preferences, architectural insights, or showcase flair discussed during this session (Living Document Protocol).
3. Update Section 6 of PROJECT_HANDOFF.md with the exact next Phase Gate for the incoming AI.
4. Commit the updated document to Git with a structured commit message.
5. Output the 1-sentence kickoff prompt for me to paste into the new chat. 
---------------------------------------------
1-Sentence Kickoff Prompt (For Your New Chat)
(Paste this into your new Antigravity chat panel):

We are continuing development of the AMR project. Read PROJECT_HANDOFF.md in the workspace root to ingest our complete architecture, calibrated parameters, operating rules, and current phase gate, then begin work on Section 6. -->
