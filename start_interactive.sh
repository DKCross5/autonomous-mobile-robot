#!/usr/bin/env bash
# ==============================================================================
# AMR Digital Twin - Interactive Multi-Terminal Session (tmux)
# ==============================================================================
# Launches all 3 AMR nodes in a single, interactive 3-pane dashboard:
#   - Pane 0 (Left):         MuJoCo Simulation Engine (sim_test.py)
#   - Pane 1 (Top-Right):    RViz2 Digital Twin (launch_rviz.sh -> amr_view.rviz)
#   - Pane 2 (Bottom-Right): Interactive Teleop & Telemetry HUD (teleop.py)
#
# Key Advantages of Interactive Terminals:
#   1. Every pane is a genuine, persistent bash shell.
#   2. If you press Ctrl+C or a script crashes, the prompt stays open!
#   3. You can press [Up Arrow] + [Enter] to immediately restart any single node.
#   4. Full mouse support enabled (click to focus pane, scroll to view history).
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION_NAME="amr_dev"

# Ensure ROS 2 Humble is available
if [ -z "$ROS_DISTRO" ] && [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux could not be found. Please install with: sudo apt install tmux"
    exit 1
fi

# If session already exists, attach to it
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    echo "Attaching to existing AMR interactive session: $SESSION_NAME"
    exec tmux attach-session -t "$SESSION_NAME"
fi

echo "Creating new interactive AMR multi-terminal session: $SESSION_NAME..."

# 1. Create a new detached session with the Simulation Node
tmux new-session -d -s "$SESSION_NAME" -n "AMR-Dashboard" -c "$SCRIPT_DIR" \
    "bash -c 'source /opt/ros/humble/setup.bash && python3 sim_test.py; exec bash'"

# Enable mouse mode for easy clicking and scrollback
tmux set-option -t "$SESSION_NAME" -g mouse on

# 2. Split horizontally to create right column for RViz2
tmux split-window -h -t "$SESSION_NAME:AMR-Dashboard" -c "$SCRIPT_DIR" \
    "bash -c 'source /opt/ros/humble/setup.bash && ./launch_rviz.sh; exec bash'"

# 3. Split the right column vertically to create bottom-right pane for Teleop HUD
tmux split-window -v -t "$SESSION_NAME:AMR-Dashboard.1" -c "$SCRIPT_DIR" \
    "bash -c 'source /opt/ros/humble/setup.bash && python3 teleop.py; exec bash'"

# Focus back on Pane 0 (Simulation) or Pane 2 (Teleop)
tmux select-pane -t "$SESSION_NAME:AMR-Dashboard.2"

echo "Launching interactive dashboard..."
exec tmux attach-session -t "$SESSION_NAME"
