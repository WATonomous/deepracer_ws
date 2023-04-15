from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='online_async',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'base_link'},
                {'scan_topic': '/rplidar/scan'},
                {'mode': 'mapping'},
                {'resolution': 0.05},
                {'max_laser_range': 20.0},
                {'minimum_time_interval': 0.5},
                {'transform_publish_period': 0.05},
                {'update_rate': 1.0}
            ]
        )
    ])