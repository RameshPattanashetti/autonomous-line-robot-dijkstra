import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg   = get_package_share_directory('line_robot')
    world = os.path.join(pkg, 'worlds',      'track.sdf')
    urdf  = os.path.join(pkg, 'description', 'robot.urdf')
    cfg   = os.path.join(pkg, 'config',      'bridge.yaml')

    with open(urdf) as f:
        robot_desc = f.read()

    # 1. Gazebo Harmonic
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    # 2. Robot state publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    # 3. Spawn robot at x=-3.5, y=0, facing East (yaw=0).
    # Load directly from the installed URDF file so spawning does not depend
    # on timing of the /robot_description topic.
    spawn = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name','line_robot','-file', urdf,
                   '-x','-3.5','-y','0','-z','0.05','-Y','0.0'],
        output='screen')])

    # 4. ROS<->Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={cfg}'],
        output='screen')

    # 5. Follower — starts after 12s (Gazebo loaded + robot spawned)
    follower = TimerAction(period=12.0, actions=[Node(
        package='line_robot',
        executable='follower',
        name='follower',
        output='screen',
        parameters=[{'use_sim_time': True}])])

    return LaunchDescription([gazebo, rsp, spawn, bridge, follower])
