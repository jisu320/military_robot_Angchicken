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
            package='pump_track',  # usb camera open
            executable='usb_cam_2',
            name='usb_2camera_node',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',  # marker_detection
            executable='marker_detect',
            name='marker_detection',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

        Node(
            package='pump_track',  # open depth cam
            executable='depth_cam',
            name='depth_camera',
            namespace=[LaunchConfiguration('namespace')],
            output='screen',
        ),

    ])