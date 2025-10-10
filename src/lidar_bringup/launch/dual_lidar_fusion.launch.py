import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch.substitutions import Command, PathJoinSubstitution
# from launch_ros.parameter_descriptions import ParameterValue  # (지금은 불필요)

def generate_launch_description():
    lidar_bringup_share = get_package_share_directory('lidar_bringup')
    sick_share = get_package_share_directory('sick_scan_xd')
    sick_multiscan_launch = os.path.join(sick_share, 'launch', 'sick_multiscan.launch.py')

    fusion_cfg = PathJoinSubstitution([
        get_package_share_directory('pointcloud_fusion'),
        'config',
        'self_filter.yaml'
    ])

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
                'publish_frame_id': 'lidar_front',
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
                    'frameid=lidar_front publish=1'
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
                'publish_frame_id': 'lidar_back',
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
                    'frameid=lidar_back publish=1'
                ),
            }.items()
        )
    ])

    # robot_state_publisher (URDF → TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'cat ',
                PathJoinSubstitution([lidar_bringup_share, 'urdf', 'lmd.urdf'])
            ])
        }],
        output='screen'
    )

    # map → base_link (optional)
    map_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        name='map_to_base_link_tf'
    )

    # Fusion Node (★ YAML 연결)
    fusion_node = Node(
        package='pointcloud_fusion',
        executable='pointcloud_fusion_node',
        name='pointcloud_fusion',
        output='screen',
        parameters=[fusion_cfg]   # ★ 여기
    )

    # RViz
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2')

    return LaunchDescription([
        robot_state_publisher,
        lidar1,
        lidar2,
        map_to_base_link,
        fusion_node,
        rviz
    ])