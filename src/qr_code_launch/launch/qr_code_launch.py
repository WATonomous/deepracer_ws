import launch
import launch_ros.actions

# colcon build --packages-select qr_code_detector qr_msgs qr_code_ctrl qr_code_launch deepracer_interfaces_pkg
def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='qr_code_detector',
            namespace='qr_code_detector',
            executable='qr_code_detector_node',
            name='qr_code_detector',
            parameters=[
                {'subscriber_topic': '/camera_pkg/display_mjpeg'},
                {'publisher_topic': '/annotated_image'},
            ]
        ),
        Node(
            package='qr_code_ctrl',
            namespace='qr_code_ctrl',
            executable='qr_code_ctrl_node',
            name='qr_code_ctrl',
            parameters=[
                {'subscriber_topic': '/camera_pkg/display_mjpeg'},
                {'publisher_topic': '/annotated_image'},
            ]
        ),
  ])