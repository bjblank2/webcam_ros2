from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node(context):
    # Get argument values
    camera_id = LaunchConfiguration('camera_id').perform(context)
    topic_name = LaunchConfiguration('topic_name').perform(context)
    camera_ids_str = LaunchConfiguration('camera_ids').perform(context)
    camera_names_str = LaunchConfiguration('camera_names').perform(context)
    
    # Parse camera_ids - use simple argument if advanced one is not provided
    if camera_ids_str:
        # Parse string representation of list (e.g., "[0]" or "[0,1]")
        import ast
        try:
            camera_ids_param = ast.literal_eval(camera_ids_str)
        except:
            camera_ids_param = [int(camera_ids_str)]
    else:
        camera_ids_param = [int(camera_id)]
    
    # Parse camera_names - use simple argument if advanced one is not provided
    if camera_names_str:
        # Parse string representation of list (e.g., '["camera"]' or '["front","back"]')
        import ast
        try:
            camera_names_param = ast.literal_eval(camera_names_str)
        except:
            camera_names_param = [camera_names_str]
    else:
        camera_names_param = [topic_name]
    
    # Ensure they are lists
    if not isinstance(camera_ids_param, list):
        camera_ids_param = [camera_ids_param]
    if not isinstance(camera_names_param, list):
        camera_names_param = [camera_names_param]
    
    return [
        Node(
            package='webcam_ros2',
            executable='webcam_ros2',
            name='webcam_ros2_node',
            parameters=[{
                'camera_ids': camera_ids_param,
                'camera_names': camera_names_param,
                'frame_rate': LaunchConfiguration('frame_rate'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
            }],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Simple arguments for single camera
        DeclareLaunchArgument(
            'camera_id',
            default_value='2',
            description='Camera device ID (e.g., 0, 1, 2)'
        ),
        DeclareLaunchArgument(
            'topic_name',
            default_value='camera',
            description='Topic name for the camera (e.g., camera, front_cam, back_cam)'
        ),
        # Advanced arguments for multiple cameras (optional)
        DeclareLaunchArgument(
            'camera_ids',
            default_value='',
            description='List of camera device IDs as string (e.g., "[0]" or "[0,1]"). If provided, overrides camera_id.'
        ),
        DeclareLaunchArgument(
            'camera_names',
            default_value='',
            description='List of camera names as string (e.g., \'["camera"]\' or \'["front_cam","back_cam"]\'). If provided, overrides topic_name.'
        ),
        DeclareLaunchArgument(
            'frame_rate',
            default_value='30.0',
            description='Camera frame rate (FPS)'
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
        
        OpaqueFunction(function=create_node),
    ])

