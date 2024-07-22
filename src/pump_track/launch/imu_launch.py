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
            package='pump_track',  # imu values
            executable='imu',
            name='imu_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),
        Node(
            package='pump_track',  # ride node
            executable='track_imu',
            name='track_imu_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),
    ])
