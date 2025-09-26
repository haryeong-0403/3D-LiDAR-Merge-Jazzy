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

    # LiDAR 1 설정 (변경 없음)
    lidar1 = GroupAction([
        PushRosNamespace('lidar1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sick_multiscan_launch),
            launch_arguments={
                'hostname': '192.168.0.160',
                'udp_receiver_ip': '192.168.0.170',
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

    # LiDAR 2 설정 (변경 없음)
    lidar2 = GroupAction([
        PushRosNamespace('lidar2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sick_multiscan_launch),
            launch_arguments={
                'hostname': '192.168.0.2',
                'udp_receiver_ip': '192.168.0.170',
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

    # --- Static TFs (좌표 변환) ---

    # map -> base_link (월드 좌표계와 로봇 바닥의 관계, 보통은 0,0,0)
    map_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        name='map_to_base_link_tf'
    )
    
    # Z축으로 38cm(0.38m)만큼 위로 이동하는 새로운 TF를 정의합니다.
    base_to_base_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.38', '0', '0', '0', 'base_link', 'base_scan'],
        name='base_to_base_scan_tf'
    )
    
    # 뒤로 23cm(x=-0.23), 왼쪽으로 20cm(y=0.20) 이동하고, 왼쪽으로 48.12도(yaw=0.840) 회전합니다.
    static_tf_lidar1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        # arguments 순서: x     y      z    yaw      pitch  roll   parent     child
        arguments=['-0.23', '0.20', '0', '-0.840', '0',   '0', 'base_scan', 'lidar1_link'],
        name='static_tf_lidar1'
    )
    
    # 이전에 보정한 최종 값을 그대로 사용합니다.
    static_tf_lidar2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            TextSubstitution(text='-0.231'), # x
            TextSubstitution(text='-0.584'), # y
            TextSubstitution(text='0.027'),  # z
            TextSubstitution(text='-0.440'), # yaw
            TextSubstitution(text='0.021'),  # pitch
            TextSubstitution(text='0.004'),  # roll
            TextSubstitution(text='base_scan'), 
            TextSubstitution(text='lidar2_link')
        ],
        name='static_tf_lidar2'
    )
    
    # RViz 노드
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2')

    return LaunchDescription([
        lidar1,
        lidar2,
        map_to_base_link,
        base_to_base_scan,    
        static_tf_lidar1,
        static_tf_lidar2,
        rviz
    ])