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
    PythonExpression,
    Command
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    balltze_description = get_package_share_directory('balltze_description')
    xacro_file = PathJoinSubstitution([balltze_description, 'urdf', 'balltze.urdf.xacro'])
    rviz_config = PathJoinSubstitution([balltze_description, 'rviz', 'model_preveiw.rviz'])

    robot_description = {
        'robot_description' : Command([
        'xacro --verbosity 0 ', xacro_file,
        ' use_sim:=true',
        ' use_gpu:=false'
        ])
    }

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Ign - ROS Bridge
    # clock_bridge = Node(
    #   package='ros_ign_bridge',
    #   executable='parameter_bridge',
    #   name='clock_bridge',
    #   arguments=[
    #     '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
    #   ],
    #   output='screen'
    # )
    
    # tf_bridge = Node(
    #   package='ros_ign_bridge',
    #   executable='parameter_bridge',
    #   name='tf_bridge',
    #   arguments=[
    #     '/tf' + '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
    #   ],
    #   output='screen'
    # )

  
    ignition_spawn_entity = Node(
      package='ros_ign_gazebo',
      executable='create',
      arguments=[
        '-name', 'balltze',
        '-allow_renaming', 'true',
        '-topic', 'robot_description'
      ],
      output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config,
                              description='Absolute path to rviz config file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py'])]),
            launch_arguments=[('ign_args', [' -r -v 4 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        robot_state_publisher_node,
        ignition_spawn_entity,

        # rviz_node
    ])