<h1>3D LiDAR Merge</h1>

# 📦 Build 명령어

```bash
colcon build --symlink-install
source install/setup.bash
```

# 🏗️ Run

## Dual LiDAR Fusion

- 명령어

```bash
ros2 launch lidar_bringup dual_lidar_fusion.launch.py
```

- 파일 위치
```bash
lidar_bringup/dual_lidar_fusion.launch.py
```

## LiDAR Object Detection(Clustering)
```bash
ros2 launch lidar_cluster euclidean_spatial.launch.py topic:=/merged_points tolerance:=0.6 verbose1:=True
```
