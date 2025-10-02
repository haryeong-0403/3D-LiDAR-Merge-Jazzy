import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    sick_share = get_package_share_directory('sick_scan_xd')
    sick_multiscan_launch = os.path.join(sick_share, 'launch', 'sick_multiscan.launch.py')

    # LiDAR 1
    lidar1 = GroupAction([
        PushRosNamespace('lidar1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sick_multiscan_launch),
            launch_arguments={
                'hostname': '192.168.127.40',
                'udp_receiver_ip': '192.168.127.30',
                'udp_port': '2115',
                'check_udp_receiver_port': '2116',
                'imu_udp_port': '7503',
                'nodename': 'multiScan160',
                'publish_frame_id': 'lidar1_link',
                'tf_publish_rate': '0.0',
                'use_sopas_startup_cmds': 'false',
                'set_udp_ip_port': 'false',
                'sopas_tcp_port': '2111',
                'custom_pointclouds': 'cloud_unstructured_fullframe',
                'cloud_unstructured_fullframe': (
                    'coordinateNotation=0 updateMethod=0 '
                    'echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 '
                    'reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 '
                    'topic=/lidar1/cloud_unstructured_fullframe '
                    'frameid=lidar1_link publish=1'
                ),
            }.items()
        )
    ])

    # LiDAR 2
    lidar2 = GroupAction([
        PushRosNamespace('lidar2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sick_multiscan_launch),
            launch_arguments={
                'hostname': '192.168.127.50',
                'udp_receiver_ip': '192.168.127.30',
                'udp_port': '2117',
                'check_udp_receiver_port': '2118',
                'imu_udp_port': '7504',
                'nodename': 'multiScan002',
                'publish_frame_id': 'lidar2_link',
                'tf_publish_rate': '0.0',
                'use_sopas_startup_cmds': 'false',
                'set_udp_ip_port': 'false',
                'sopas_tcp_port': '2112',
                'custom_pointclouds': 'cloud_unstructured_fullframe',
                'cloud_unstructured_fullframe': (
                    'coordinateNotation=0 updateMethod=0 '
                    'echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 '
                    'reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 '
                    'topic=/lidar2/cloud_unstructured_fullframe '
                    'frameid=lidar2_link publish=1'
                ),
            }.items()
        )
    ])

    # TF 설정
    map_to_base_link = Node( # map 좌표계와 base_link 좌표계를 일치시킴
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        name='map_to_base_link_tf'
    )
    
    base_to_base_scan = Node( # base_scan
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.38', '0', '0', '0', 'base_link', 'base_scan'],
        name='base_to_base_scan_tf'
    )
    
    """
    x (m)

    y (m)

    z (m)

    yaw (rad)

    pitch (rad)

    roll (rad)

    parent frame

    child frame
    """
    static_tf_lidar1 = Node( # base_scan → lidar1_link (첫 번째 LiDAR의 위치/자세)
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['-0.23', '0.20', '0', '-0.840', '0', '0', 'base_scan', 'lidar1_link'],
        name='static_tf_lidar1'
    )
    
    static_tf_lidar2 = Node( 
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            TextSubstitution(text='-0.231'),
            TextSubstitution(text='-0.584'),
            TextSubstitution(text='0.027'),
            TextSubstitution(text='-0.440'),
            TextSubstitution(text='0.021'),
            TextSubstitution(text='0.004'),
            TextSubstitution(text='base_scan'),
            TextSubstitution(text='lidar2_link')
        ],
        name='static_tf_lidar2'
    )

    # Fusion Node 실행
    fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion',
        output='screen'
    )
    
    # RViz 실행
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2')

    return LaunchDescription([
        lidar1,
        lidar2,
        map_to_base_link,
        base_to_base_scan,    
        static_tf_lidar1,
        static_tf_lidar2,
        fusion_node,
        rviz
    ])
