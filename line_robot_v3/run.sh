#!/bin/bash
# ════════════════════════════════════════════════════════════
#  Line Robot — Dijkstra Shortest-Path Edition
#  Ubuntu 24.04 / ROS2 Jazzy / Gazebo Harmonic
#  Usage:  bash run.sh
# ════════════════════════════════════════════════════════════
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$HOME/ros2_ws"

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Line Follower Robot  —  Dijkstra Shortest-Path Edition"
echo "  Ubuntu 24 / ROS2 Jazzy / Gazebo Harmonic"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "  NEW: Dijkstra's algorithm pre-plans the shortest route at"
echo "  startup, assigning obstacle penalties to graph nodes so"
echo "  the SHORTCUT path (above the obstacles) is chosen."
echo "  The robot uses waypoint navigation until it sees the"
echo "  shortcut black line, then switches to camera P-control."
echo ""

# 1. Source ROS2
source /opt/ros/jazzy/setup.bash

# 2. Install dependencies
echo "[1/4] Installing dependencies..."
sudo apt-get install -y -q \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-robot-state-publisher \
    python3-opencv \
    python3-numpy 2>/dev/null || true

# 3. Copy package into workspace
echo "[2/4] Setting up workspace..."
mkdir -p "$WS/src"
rm -rf "$WS/src/line_robot"
cp -r "$DIR/line_robot" "$WS/src/"

# 4. Build
echo "[3/4] Building..."
cd "$WS"
colcon build --packages-select line_robot --symlink-install
echo "      Build complete!"

# 5. Source workspace
source "$WS/install/setup.bash"

echo "[4/4] Launching..."
echo ""
echo "════════════════════════════════════════════════════════════"
echo "  Gazebo opens → robot spawns at 5 s"
echo "  At 12 s: Dijkstra runs, chosen route is logged to console"
echo "  Robot navigates waypoints → locks on shortcut line"
echo "  Yellow marker posts show the shortcut corridor"
echo "  Press Ctrl+C to stop"
echo "════════════════════════════════════════════════════════════"
echo ""

ros2 launch line_robot simulation.launch.py
