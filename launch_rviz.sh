#!/usr/bin/env bash
# ==============================================================================
# AMR RViz2 Automated Visualization Launcher
# ==============================================================================
# Automatically sources ROS 2 Humble and launches RViz2 with the pre-configured
# AMR digital twin profile (Fixed Frame: odom, TF, Robot Markers, Path, Range, Odometry).
# ==============================================================================

set -e

# Determine directory where this script resides
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/amr_view.rviz"

# Terminal ANSI styling
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m' # No Color

echo -e "${CYAN}${BOLD}================================================================${NC}"
echo -e "${CYAN}${BOLD}       🚀 AMR Digital Twin - RViz2 Automated Launcher           ${NC}"
echo -e "${CYAN}${BOLD}================================================================${NC}"

# 1. Source ROS 2 Humble if not already in environment
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${YELLOW}ℹ ROS 2 environment not detected. Sourcing /opt/ros/humble/setup.bash...${NC}"
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}❌ Error: ROS 2 Humble setup.bash not found at /opt/ros/humble/setup.bash${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✔ ROS 2 ($ROS_DISTRO) environment active.${NC}"
fi

# 2. Check configuration file
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ Error: RViz configuration file not found at: ${CONFIG_FILE}${NC}"
    exit 1
fi

echo -e "${GREEN}✔ Loading Configuration:${NC} ${CONFIG_FILE}"
echo -e "${CYAN}  • Fixed Frame   :${NC} odom"
echo -e "${CYAN}  • 3D Robot Mesh :${NC} /robot_visual (MarkerArray)"
echo -e "${CYAN}  • Trajectory    :${NC} /robot_path (Path, Neon Green)"
echo -e "${CYAN}  • Sensor Cone   :${NC} /sim_lidar (Range, Cyan)"
echo -e "${CYAN}  • Odometry Pose :${NC} /odom (Arrow, Orange)"
echo -e "${CYAN}  • Coordinate TF :${NC} odom -> base_link -> lidar_link"
echo -e "${CYAN}${BOLD}----------------------------------------------------------------${NC}"
echo -e "${YELLOW}Starting RViz2... (Press Ctrl+C to close)${NC}\n"

# 3. Launch RViz2 with the display configuration and pass through any CLI arguments
exec rviz2 -d "$CONFIG_FILE" "$@"
