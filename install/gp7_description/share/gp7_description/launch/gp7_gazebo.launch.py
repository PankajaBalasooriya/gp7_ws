# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, TimerAction
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     pkg_path = get_package_share_directory('gp7_description')

#     world_path = os.path.join(pkg_path, 'worlds', 'gp7_world.sdf')
#     urdf_path = os.path.join(pkg_path, 'urdf', 'gp7.urdf')

#     # 1. Start Gazebo with your custom world
#     gazebo = ExecuteProcess(
#         cmd=[
#             'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
#             f'gz_args:=-r {world_path}'
#         ],
#         output='screen'
#     )

#     # 2. Robot State Publisher
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': open(urdf_path).read()}]
#     )

#     # 3. Spawn robot into Gazebo after a short delay
#     spawn_robot = TimerAction(
#         period=5.0,
#         actions=[
#             ExecuteProcess(
#                 cmd=[
#                     'ros2', 'run', 'ros_gz_sim', 'create',
#                     '-name', 'gp7',
#                     '-x', '0', '-y', '0', '-z', '0',
#                     '-topic', '/robot_description'
#                 ],
#                 output='screen'
#             )
#         ]
#     )

#     # 4. Load controllers after Gazebo + plugin are running
#     load_joint_state_broadcaster = TimerAction(
#         period=10.0,
#         actions=[
#             ExecuteProcess(
#                 cmd=[
#                     'ros2', 'control', 'load_controller',
#                     '--set-state', 'active',
#                     'joint_state_broadcaster'
#                 ],
#                 output='screen'
#             )
#         ]
#     )

#     load_trajectory_controller = TimerAction(
#         period=11.0,
#         actions=[
#             ExecuteProcess(
#                 cmd=[
#                     'ros2', 'control', 'load_controller',
#                     '--set-state', 'active',
#                     'gp7_trajectory_controller'
#                 ],
#                 output='screen'
#             )
#         ]
#     )

#     return LaunchDescription([
#         gazebo,
#         robot_state_publisher,
#         spawn_robot,
#         load_joint_state_broadcaster,
#         load_trajectory_controller
#     ])


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
    gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
            f'gz_args:=-r {world_path}'
        ],
        output='screen'
    )

    # 2. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # 3. Spawn robot into Gazebo after a short delay
    spawn_robot = TimerAction(
        period=5.0,
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
        rsp,
        spawn_robot,
    ])
