from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  ExecuteProcess,
  IncludeLaunchDescription,
  RegisterEventHandler
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    Command
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    balltze_description = get_package_share_directory('balltze_description')
    balltze_bringup = get_package_share_directory('balltze_bringup')
    xacro_file = PathJoinSubstitution([balltze_description, 'urdf', 'balltze.urdf.xacro'])
    rviz_config = PathJoinSubstitution([balltze_description, 'rviz', 'model_preveiw.rviz'])

    robot_description = {
        'robot_description' : Command([
        'xacro --verbosity 0 ', xacro_file,
        ' use_sim:=false',
        ' use_gpu:=false'
        ])
    }

    balltze_bringup = get_package_share_directory('balltze_bringup')
    champ_confgi = PathJoinSubstitution([balltze_bringup, 'config', 'controllers.yaml'])

    robot_controllers = PathJoinSubstitution([balltze_bringup, 'config', 'controllers.yaml'])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # rqt_joint_trajectory_controller = Node(
    #     package='rqt_joint_trajectory_controller',
    #     executable='rqt_joint_trajectory_controller',
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config,
                              description='Absolute path to rviz config file'),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[rviz_node],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=robot_controller_spawner,
        #         on_exit=[rqt_joint_trajectory_controller],
        #     )
        # ),
        joint_state_broadcaster_spawner,
        robot_state_publisher_node,
        control_node,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])