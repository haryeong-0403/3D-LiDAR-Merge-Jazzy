from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_fusion',
            executable='fusion_node',
            name='pointcloud_fusion_node',
            output='screen',
            parameters=[{
                'target_frame': 'base_link' # 사용할 공통 좌표계
            }],
            remappings=[
                # 코드 상의 토픽 이름 -> 실제 사용하는 토픽 이름
                ('/lidar1/points', '/lidar1/cloud_unstructured_fullframe'),
                ('/lidar2/points', '/lidar2/cloud_unstructured_fullframe'),
                ('/merged_points', '/points_fused') # 병합된 결과가 발행될 토픽 이름
            ]
        )
    ])