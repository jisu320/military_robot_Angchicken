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
            package='pump_track',  # diffbot control
            executable='diffbot_cont',
            name='diffbot_control',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',  # ride track
            executable='track_fall_check',
            name='track_check_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',  # usb camera open
            executable='usb_cam_2',
            name='usb_2camera_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),
        
        Node(
            package='pump_track',  # marker detect
            executable='marker_detect',
            name='marker_detection',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

    ])
