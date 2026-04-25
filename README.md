# Line Robot Dijkstra v2

ROS 2 Jazzy and Gazebo Harmonic line-following robot that pre-plans a route with Dijkstra's algorithm, then follows the visible track with camera feedback and falls back to LiDAR-based obstacle bypass when needed.

The project is packaged as a ROS 2 Python package under `line_robot_v3/line_robot`. A helper script in `line_robot_v3/run.sh` installs dependencies, copies the package into `~/ros2_ws`, builds it with `colcon`, and launches the simulation.

## Features

- Dijkstra pre-planning over a small waypoint graph before motion starts
- Obstacle-aware route cost using penalties around known obstacle locations
- Camera-based black-line following with proportional steering control
- Waypoint navigation until the planned line corridor is detected
- Reactive LiDAR bypass for unexpected obstacles during execution
- Gazebo Harmonic world with a multi-branch track, obstacles, and finish wall

## Repository Layout

```text
.
├── README.md
└── line_robot_v3
    ├── run.sh
    └── line_robot
        ├── config/bridge.yaml
        ├── description/robot.urdf
        ├── launch/simulation.launch.py
        ├── line_robot/
        │   ├── follower.py
        │   └── path_planner.py
        ├── package.xml
        ├── setup.py
        └── worlds/track.sdf
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic / `ros_gz`
- Python packages used by the node:
  - `opencv-python` via `python3-opencv`
  - `numpy`

The package metadata in [`line_robot_v3/line_robot/package.xml`](/home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot/package.xml) declares:

- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `nav_msgs`
- `python3-opencv`
- `ros_gz_sim`
- `ros_gz_bridge`

## Quick Start

### Option 1: Use the helper script

This is the intended path for a fresh local machine:

```bash
cd /home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3
bash run.sh
```

What `run.sh` does:

1. Sources `/opt/ros/jazzy/setup.bash`
2. Installs required apt packages
3. Copies `line_robot_v3/line_robot` into `~/ros2_ws/src/line_robot`
4. Builds the package with `colcon build --packages-select line_robot --symlink-install`
5. Launches `ros2 launch line_robot simulation.launch.py`

Note: the script uses `sudo apt-get install` and deletes any previous `~/ros2_ws/src/line_robot` copy before replacing it.

### Option 2: Build and launch manually

If you already have a ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ros2_ws/src
cp -r /home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select line_robot --symlink-install
source install/setup.bash
ros2 launch line_robot simulation.launch.py
```

## Runtime Behavior

### 1. Path planning

[`path_planner.py`](/home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot/line_robot/path_planner.py) builds a fixed graph in the odometry frame and runs Dijkstra from `start` to `end`.

- The graph contains two route choices between the first and second junctions:
  - `MAIN LINE`
  - `SHORTCUT`
- Known obstacle positions add a cost penalty to nearby nodes.
- The selected route is returned as odometry-frame waypoints plus total plan cost.

In the current world, the planner is designed to prefer the shortcut branch because it clears the first obstacle better than the main zig-zag branch.

### 2. Follower state machine

[`follower.py`](/home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot/line_robot/follower.py) runs a ROS 2 node named `follower` and combines three sensing/control modes:

- Waypoint navigation:
  - Starts from the Dijkstra waypoints
  - Steers toward the next waypoint using odometry and a heading controller
- Camera line following:
  - Thresholds the camera image for the black track
  - Uses centroid error and a proportional controller for steering
- Reactive bypass:
  - Uses front LiDAR distance to detect an unexpected blockage
  - Executes a right-side detour in odometry space
  - Reacquires and recenters on the line before resuming normal tracking

The robot stops when it reaches the end wall near the finish pad.

## Launch Sequence

[`simulation.launch.py`](/home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot/launch/simulation.launch.py) performs the following:

1. Starts Gazebo with `worlds/track.sdf`
2. Starts `robot_state_publisher`
3. Spawns the robot from `description/robot.urdf` after 5 seconds
4. Starts `ros_gz_bridge` using `config/bridge.yaml`
5. Starts the `follower` node after 12 seconds

The robot spawns at world coordinates `(-3.5, 0.0)` facing east. The odometry frame is defined so that this spawn point becomes `(0, 0)`.

## ROS Interfaces

The ROS-Gazebo bridge in [`bridge.yaml`](/home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot/config/bridge.yaml) maps these topics:

- `/cmd_vel` as `geometry_msgs/msg/Twist` from ROS to Gazebo
- `/scan` as `sensor_msgs/msg/LaserScan` from Gazebo to ROS
- `/camera/image_raw` as `sensor_msgs/msg/Image` from Gazebo to ROS
- `/odom` as `nav_msgs/msg/Odometry` from Gazebo to ROS
- `/clock` as `rosgraph_msgs/msg/Clock` from Gazebo to ROS

## Simulation World

The Gazebo world in [`track.sdf`](/home/rameshpattanashetti/line_robot_dijkstra_v2/line_robot_v3/line_robot/worlds/track.sdf) includes:

- A white floor with a black multi-segment track
- Start and finish pads to help line acquisition
- Two cylindrical obstacles placed near the route
- Extra visual markers around the course

The track geometry matches the waypoint graph used by the planner, including the branch where the shortcut diverges and merges back.

## Development Notes

- Package name: `line_robot`
- Entry point: `ros2 run line_robot follower`
- Build type: `ament_python`
- Main code files:
  - Planner: `line_robot_v3/line_robot/line_robot/path_planner.py`
  - Controller: `line_robot_v3/line_robot/line_robot/follower.py`
  - Launch: `line_robot_v3/line_robot/launch/simulation.launch.py`

## Troubleshooting

- If `ros2 launch line_robot simulation.launch.py` cannot find the package, source both ROS 2 and your workspace `install/setup.bash`.
- If Gazebo launches but the robot does not move, verify that `/scan`, `/odom`, and `/camera/image_raw` are being bridged.
- If the helper script fails during dependency installation, run the `apt-get` step manually and rerun the build.
- If the camera loses the line for too long, the controller falls back to search and reacquisition behavior; check the world lighting and camera topic.

## License

The package metadata declares the project license as Apache-2.0.
