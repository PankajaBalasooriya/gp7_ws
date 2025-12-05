from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('gp7_description')

    world_path = os.path.join(pkg_path, 'worlds', 'gp7_world.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'gp7.urdf')

    # 1. Start Gazebo with world
    gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
            f'gz_args:={world_path}'
        ],
        output='screen'
    )

    # 2. Robot state publisher (publishes /robot_description)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # 3. Spawn robot into Gazebo at origin, using /robot_description
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'gp7',
            '-x', '0', '-y', '0', '-z', '0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # NOTE:
    # - We do NOT start ros2_control_node here.
    # - The ros2_control controller_manager is created INSIDE Gazebo
    #   by the gz_ros_control / gz_ros2_control plugin using <ros2_control> tag in URDF.
    # - We will load controllers using `ros2 control` CLI.

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
