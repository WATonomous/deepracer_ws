from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mono_depth_estimation',
            namespace='mono_depth_estimation',
            executable='mono_depth_estimation_node',
            name='mono_depth_estimation',
            parameters=[
                {'subscriber_topic': '/camera_pkg/video_mjpeg'},
                {'publisher_topic': '/mono_depth_map'},
                {'model_path': '/perception_models/model-small.pb'},
                {'model_config_path': ''}, # only needed for OpenVINO
            ]
        )
    ])
