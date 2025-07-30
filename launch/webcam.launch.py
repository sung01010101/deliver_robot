from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    ## ***** Launch arguments *****
    view_vid = DeclareLaunchArgument('view_vid', default_value='True')
    device_id = DeclareLaunchArgument('device_id', default_value='0')

    # Webcam publisher (publishes /image)
    webcam_node = Node(
        package='image_tools',
        executable='cam2image',
        name='cam2image',
        parameters=[
            {'width': 640},
            {'height': 480},
            {'framerate': 30},
            {'device_id': LaunchConfiguration('device_id')}
        ],
        output='screen'
    )

    # Image viewer (subscribes to /image) - only launch if view_vid is True
    image_viewer_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        remappings=[('/image', '/image')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('view_vid'))
    )
    
    return LaunchDescription([
        view_vid,
        device_id,
        webcam_node,
        image_viewer_node
    ])
