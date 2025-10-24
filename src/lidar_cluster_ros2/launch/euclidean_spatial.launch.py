from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_cluster')  
    params_file = os.path.join(pkg_share, 'config', 'euclidean_spatial.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            "topic",
            description="a pointcloud topic to process",
            default_value="/merged_points"
        ),
        Node(
            package='lidar_cluster',                 
            executable='euclidean_spatial',
            output='screen',
            parameters=[
                params_file,                         # share/lidar_cluster/config/euclidean_spatial.yaml
                {'points_in_topic': LaunchConfiguration("topic")},
            ]
        )
    ])
