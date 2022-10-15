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

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim', default=False)

    balltze_description = get_package_share_directory('balltze_description')
    balltze_bringup = get_package_share_directory('balltze_bringup')

    xacro_file = PathJoinSubstitution([balltze_description, 'urdf', 'balltze.urdf.xacro'])
    champ_config = PathJoinSubstitution([balltze_bringup, 'config', 'champ.yaml'])
    joint_offset_config = PathJoinSubstitution([balltze_bringup, 'config', 'relay_joint_offsets.yaml'])

    robot_description_string = {
        'urdf' : Command([
        'xacro --verbosity 0 ', xacro_file,
        ' use_sim:=', use_sim,
        ' use_gpu:=false'
        ])
    }

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            robot_description_string,
            champ_config
        ],
        remappings=[
            ("/cmd_vel/smooth", "/cmd_vel"),
            ("/joint_states", "/joint_states/offset")
        ],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            robot_description_string,
            champ_config
        ],
        remappings=[
            ("/joint_states", "/joint_states/offset")
        ],
    )

    message_realy_node = Node(
        package="balltze_msgs",
        executable="message_relay_node",
        output="screen",
        parameters=[
            joint_offset_config
        ],
        remappings=[
            ("/joint_trajectory", "/champ/joint_trajectory"),
            ("/joint_trajectory/offset", "/joint_trajectory_controller/joint_trajectory")
        ],
    )

    return LaunchDescription([
        quadruped_controller_node,
        state_estimator_node,
        message_realy_node
    ])