from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('gp7_description')

    world_path = os.path.join(pkg_path, 'worlds', 'gp7_world.sdf')
    xacro_path = os.path.join(pkg_path, 'urdf', 'gp7.urdf.xacro')
    controller_config_path = os.path.join(pkg_path, 'config', 'gp7_controllers.yaml')

    # Process xacro
    robot_description = xacro.process_file(
        xacro_path,
        mappings={'controller_config_path': controller_config_path}
    ).toxml()

    # 1. Start Gazebo with your custom world
    gazebo = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
            f'gz_args:=-r {world_path}'
        ],
        output='screen'
    )

    # 2. Bridge the clock from Gazebo to ROS2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 3. Robot State Publisher with use_sim_time
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ]
    )

    # 4. Spawn robot into Gazebo after a short delay
    spawn_robot = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-name', 'gp7',
                    '-x', '0', '-y', '0', '-z', '0',
                    '-R', '0', '-P', '0', '-Y', '3.14159',
                    '-topic', '/robot_description'
                ],
                output='screen'
            )
        ]
    )

    # 5. Load joint_state_broadcaster
    load_joint_state_broadcaster = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'joint_state_broadcaster'
                ],
                output='screen'
            )
        ]
    )

    # 6. Load arm_controller
    load_arm_controller = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'arm_controller'
                ],
                output='screen'
            )
        ]
    )

    # 7. Load gripper_controller
    load_gripper_controller = TimerAction(
        period=22.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'gripper_controller'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        rsp,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller  # Add gripper controller
    ])