from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            namespace='object_detection',
            executable='object_detection_node',
            name='object_detection',
            parameters=[
                {'subscriber_topic': '/camera_pkg/display_mjpeg'},
                {'publisher_topic': '/annotated_image'},
                # model config for OpenVINO
                {'model_path': '/perception_models/yolov5s_openvino_model/yolov5s.onnx'},
                {'model_config_path': '/perception_models/yolov5s_openvino_model/yolov5s.xml'},
                # # model config for ONNX
                # {'model_path': '/perception_models/yolov5s.onnx'},
                # {'model_config_path': ''}, # only needed for OpenVINO
                {'classes_txt_path': '/perception_models/yolov5s_openvino_model/classes.txt'}
            ]
        )
    ])