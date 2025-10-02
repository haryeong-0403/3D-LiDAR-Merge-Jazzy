import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution

def generate_launch_description():
    
    # Declare all arguments to be passed from the parent launch file
    hostname_arg = DeclareLaunchArgument('hostname', default_value='192.168.127.40')
    udp_receiver_ip_arg = DeclareLaunchArgument('udp_receiver_ip', default_value='192.168.127.30')
    nodename_arg = DeclareLaunchArgument('nodename', default_value='lidar1_link')
    publish_frame_id_arg = DeclareLaunchArgument('publish_frame_id', default_value='world')
    udp_port_arg = DeclareLaunchArgument('udp_port', default_value='2115')
    imu_udp_port_arg = DeclareLaunchArgument('imu_udp_port', default_value='7503')
    check_udp_receiver_port_arg = DeclareLaunchArgument('check_udp_receiver_port', default_value='2116')
    use_sopas_startup_cmds_arg = DeclareLaunchArgument('use_sopas_startup_cmds', default_value='true')
    set_udp_ip_port_arg = DeclareLaunchArgument('set_udp_ip_port', default_value='true')
    sopas_tcp_port_arg = DeclareLaunchArgument('sopas_tcp_port', default_value='2111')
    tf_publish_rate_arg = DeclareLaunchArgument('tf_publish_rate', default_value='10.0')
    custom_pointclouds_arg = DeclareLaunchArgument('custom_pointclouds', default_value='cloud_unstructured_fullframe cloud_polar_unstructured_fullframe')
    
    cloud_unstructured_fullframe_arg = DeclareLaunchArgument(
        'cloud_unstructured_fullframe',
        default_value=(
            'coordinateNotation=0 updateMethod=0 '
            'echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 '
            'reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 '
            'topic=/cloud_unstructured_fullframe '
            'frameid=world publish=1'
        )
    )
    cloud_polar_unstructured_fullframe_arg = DeclareLaunchArgument(
        'cloud_polar_unstructured_fullframe',
        default_value=(
            'coordinateNotation=1 updateMethod=0 '
            'echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 '
            'reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 '
            'topic=/cloud_polar_unstructured_fullframe '
            'frameid=world publish=1'
        )
    )

    # Get LaunchConfigurations for use in the Node's arguments
    hostname = LaunchConfiguration('hostname')
    udp_receiver_ip = LaunchConfiguration('udp_receiver_ip')
    nodename = LaunchConfiguration('nodename')
    publish_frame_id = LaunchConfiguration('publish_frame_id')
    udp_port = LaunchConfiguration('udp_port')
    imu_udp_port = LaunchConfiguration('imu_udp_port')
    check_udp_receiver_port = LaunchConfiguration('check_udp_receiver_port')
    use_sopas_startup_cmds = LaunchConfiguration('use_sopas_startup_cmds')
    set_udp_ip_port = LaunchConfiguration('set_udp_ip_port')
    sopas_tcp_port = LaunchConfiguration('sopas_tcp_port')
    custom_pointclouds = LaunchConfiguration('custom_pointclouds')
    cloud_unstructured_fullframe = LaunchConfiguration('cloud_unstructured_fullframe')
    cloud_polar_unstructured_fullframe = LaunchConfiguration('cloud_polar_unstructured_fullframe')
    tf_publish_rate = LaunchConfiguration('tf_publish_rate')

    sick_generic_caller_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        # name=nodename을 삭제합니다.
        output='screen',
        arguments=[
            PathJoinSubstitution([
                get_package_share_directory('sick_scan_xd'),
                'launch',
                'sick_multiscan.launch'
            ]),
            ['hostname:=', hostname],
            ['udp_receiver_ip:=', udp_receiver_ip],
            ['udp_port:=', udp_port],
            ['imu_udp_port:=', imu_udp_port],
            ['check_udp_receiver_port:=', check_udp_receiver_port],
            ['publish_frame_id:=', publish_frame_id],
            ['use_sopas_startup_cmds:=', use_sopas_startup_cmds],
            ['set_udp_ip_port:=', set_udp_ip_port],
            ['sopas_tcp_port:=', sopas_tcp_port],
            ['custom_pointclouds:=', custom_pointclouds],
            ['tf_publish_rate:=', tf_publish_rate],
            ['cloud_unstructured_fullframe:=', cloud_unstructured_fullframe],
            ['cloud_polar_unstructured_fullframe:=', cloud_polar_unstructured_fullframe],
        ],
        respawn=False
    )

    return LaunchDescription([
        hostname_arg,
        udp_receiver_ip_arg,
        nodename_arg,
        publish_frame_id_arg,
        udp_port_arg,
        imu_udp_port_arg,
        check_udp_receiver_port_arg,
        use_sopas_startup_cmds_arg,
        set_udp_ip_port_arg,
        sopas_tcp_port_arg,
        custom_pointclouds_arg,
        cloud_unstructured_fullframe_arg,
        cloud_polar_unstructured_fullframe_arg,
        sick_generic_caller_node
    ])