from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('gp7_description')

    world_path = os.path.join(pkg_path, 'worlds', 'gp7_world.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'gp7.urdf')

    # 1. Start Gazebo with your custom world
    # NOTE: gz_args must be one string: 'gz_args:="-r <world>"'
    gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
            f'gz_args:=-r {world_path}'
        ],
        output='screen'
    )

    # 2. Publish robot description to ROS
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read()
        }]
    )

    # 3. Spawn robot into Gazebo at (0,0,0) after a short delay
    spawn_robot = TimerAction(
        period=5.0,  # seconds; give Gazebo time to start
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-name', 'gp7',
                    '-x', '0', '-y', '0', '-z', '0',
                    '-topic', '/robot_description'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])
