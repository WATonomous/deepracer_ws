from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qr_code_detector',
            namespace='qr_code_detector',
            executable='qr_code_detector_node',
            name='qr_code_detector',
            parameters=[
                {'subscriber_topic': '/camera_pkg/display_mjpeg'},
                {'publisher_topic': '/annotated_image'},
            ]
        )
    ])
