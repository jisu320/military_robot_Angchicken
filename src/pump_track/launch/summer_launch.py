import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),

        Node(
            package='pump_track',
            executable='diffbot_cont',
            name='diffbot_control',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',
            executable='track_dep',
            name='track_dep_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',
            executable='airdrop_pub',
            name='airdrop_publish',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',
            executable='control',
            name='control_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),


    ])
