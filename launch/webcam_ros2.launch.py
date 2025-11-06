from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_ids',
            default_value='[0]',
            description='List of camera device IDs (e.g., [0] or [0,1])'
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='30.0',
            description='Camera frame rate (FPS)'
        ),
        DeclareLaunchArgument(
            'camera_names',
            default_value='["camera"]',
            description='List of camera names for topics (e.g., ["camera"] or ["front_cam","back_cam"])'
        ),
        DeclareLaunchArgument(
            'width',
            default_value='640',
            description='Camera image width'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='480',
            description='Camera image height'
        ),
        
        Node(
            package='webcam_ros2',
            executable='webcam_ros2',
            name='webcam_ros2_node',
            parameters=[{
                'camera_ids': LaunchConfiguration('camera_ids'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'camera_names': LaunchConfiguration('camera_names'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
            }],
            output='screen'
        ),
    ])

