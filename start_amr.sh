#!/usr/bin/env bash
# ==============================================================================
# AMR Digital Twin - Master All-in-One Launcher
# ==============================================================================
# Concurrently launches:
#   1. MuJoCo Simulation Engine (sim_test.py)
#   2. RViz2 Digital Twin Visualizer (launch_rviz.sh -> amr_view.rviz)
#   3. Interactive Teleoperation HUD & Control Pad (teleop.py)
#
# Clean shutdown:
#   Pressing Ctrl+C or closing the Teleop HUD window automatically shuts
#   down all spawned processes (no orphaned background nodes).
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Styling
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${CYAN}${BOLD}================================================================${NC}"
echo -e "${CYAN}${BOLD}       🤖 Launching AMR Digital Twin Full System                ${NC}"
echo -e "${CYAN}${BOLD}================================================================${NC}"

# Source ROS 2 Humble if needed
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${YELLOW}ℹ Sourcing /opt/ros/humble/setup.bash...${NC}"
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}❌ Error: /opt/ros/humble/setup.bash not found.${NC}"
        exit 1
    fi
fi

# Track child PIDs for clean teardown
PID_SIM=""
PID_RVIZ=""

cleanup() {
    echo -e "\n${YELLOW}Shutting down AMR Digital Twin nodes...${NC}"
    if [ -n "$PID_RVIZ" ] && kill -0 "$PID_RVIZ" 2>/dev/null; then
        kill -TERM "$PID_RVIZ" 2>/dev/null || true
    fi
    if [ -n "$PID_SIM" ] && kill -0 "$PID_SIM" 2>/dev/null; then
        kill -TERM "$PID_SIM" 2>/dev/null || true
    fi
    # Wait briefly for graceful exit
    sleep 0.5
    echo -e "${GREEN}✔ All nodes terminated cleanly.${NC}"
}

trap cleanup SIGINT SIGTERM EXIT

# 1. Start MuJoCo Simulation Node
echo -e "${GREEN}[1/3] Starting MuJoCo Physics Simulation...${NC}"
python3 "${SCRIPT_DIR}/sim_test.py" &
PID_SIM=$!

# Brief pause to allow ROS nodes and TF tree to register
sleep 1.5

# 2. Start RViz2 with pre-configured layout
echo -e "${GREEN}[2/3] Starting RViz2 Digital Twin Visualizer...${NC}"
bash "${SCRIPT_DIR}/launch_rviz.sh" &
PID_RVIZ=$!

# Brief pause before launching teleop
sleep 1.0

# 3. Start Teleop Control Pad & HUD in foreground
echo -e "${GREEN}[3/3] Starting Interactive Teleop & Telemetry HUD...${NC}"
echo -e "${CYAN}----------------------------------------------------------------${NC}"
echo -e "${CYAN}Use the GUI Control Pad or WASD keys to drive the AMR.${NC}"
echo -e "${CYAN}Closing the HUD or pressing Ctrl+C here shuts down all nodes.${NC}"
echo -e "${CYAN}----------------------------------------------------------------${NC}\n"

python3 "${SCRIPT_DIR}/teleop.py"
