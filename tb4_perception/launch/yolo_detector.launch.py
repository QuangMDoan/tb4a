import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tb4_perception')
    default_params = os.path.join(pkg_share, 'config', 'yolo_detector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the YOLO detector paramaters YAML file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'publish_visualisation',
            default_value='false',
            description='Publish annotated detection images',
        ),
        Node(
            package='tb4_perception',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'publish_visualisation': LaunchConfiguration('publish_visualisation')},
            ],
        ),
    ])