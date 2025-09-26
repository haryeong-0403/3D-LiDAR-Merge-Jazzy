#!/bin/bash

# LiDAR 1 노드 실행 (백그라운드에서 실행)
source ~/sick_scan_ws/install/setup.bash
ros2 launch sick_scan_xd sick_multiscan.launch.py \
hostname:=192.168.0.160 \
udp_receiver_ip:=192.168.0.170 \
udp_port:=2115 \
imu_udp_port:=7503 \
check_udp_receiver_port:=2116 \
nodename:=multiScan160 \
publish_frame_id:=lidar1_link \
custom_pointclouds:="cloud_unstructured_fullframe cloud_polar_unstructured_fullframe" \
cloud_unstructured_fullframe:="coordinateNotation=0 updateMethod=0 echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 topic=/lidar1/cloud_unstructured_fullframe frameid=lidar1_link publish=1" \
cloud_polar_unstructured_fullframe:="coordinateNotation=1 updateMethod=0 echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 topic=/lidar1/cloud_polar_unstructured_fullframe frameid=lidar1_link publish=1" &

# LiDAR 2 노드 실행 (백그라운드에서 실행)
source ~/sick_scan_ws/install/setup.bash
ros2 launch sick_scan_xd sick_multiscan.launch.py \
hostname:=192.168.0.2 \
udp_receiver_ip:=192.168.0.170 \
udp_port:=2117 \
imu_udp_port:=7504 \
check_udp_receiver_port:=2118 \
nodename:=multiScan002 \
publish_frame_id:=lidar2_link \
custom_pointclouds:="cloud_unstructured_fullframe cloud_polar_unstructured_fullframe" \
cloud_unstructured_fullframe:="coordinateNotation=0 updateMethod=0 echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 topic=/lidar2/cloud_unstructured_fullframe frameid=lidar2_link publish=1" \
cloud_polar_unstructured_fullframe:="coordinateNotation=1 updateMethod=0 echos=0,1,2 layers=1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 reflectors=0,1 infringed=0,1 rangeFilter=0.05,999,1 topic=/lidar2/cloud_polar_unstructured_fullframe frameid=lidar2_link publish=1" &

# 약간의 지연 시간을 줘서 노드들이 안정화될 시간을 줍니다.
sleep 5

# static_transform_publisher 노드 실행 (백그라운드에서 실행)
source ~/sick_scan_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link lidar1_link &
source ~/sick_scan_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0.7 0 0 0 0 base_link lidar2_link &

# RViz2 실행
source ~/sick_scan_ws/install/setup.bash
ros2 run rviz2 rviz2