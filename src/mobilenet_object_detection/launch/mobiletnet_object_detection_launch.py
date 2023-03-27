from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobilenet_object_detection',
            namespace='mobilenet_object_detection',
            executable='mobilenet_object_detection_node',
            name='mobilenet_object_detection',
            parameters=[
                {'subscriber_topic': '/camera_pkg/display_mjpeg'},
                {'publisher_topic': '/annotated_image'},
                {'model_path': '/perception_models/mobilenet_v3.pb'},
                {'model_config_path': '/perception_models/mobilenet_v3.pbtxt'},
                {'classes_txt_path': '/home/deepracer/Documents/deepracer_ws/src/mobilenet_object_detection/src/class_list.txt'}
            ]
        )
    ])