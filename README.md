<h1>3D LiDAR Merge</h1>

# 📦 Build 명령어

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

# 🏗️ Run

## LiDAR 두 개 동시에 실행

- 명령어

```bash
ros2 launch lidar_bringup dual_lidar.launch.py
```

- 파일 위치
lidar_bringup/dual_lidar.launch.py

# 🚀 앞으로 진행해야할 점 (Project Roadmap)

1️⃣ Extrinsic Calibration (ICP)

- 두 LiDAR 포인트 클라우드에서 겹치는 영역을 이용해 ICP 수행

- 변환 결과:

  회전행렬 R → 쿼터니언(qx, qy, qz, qw)로 변환

  이동벡터 t → (x, y, z)


2️⃣ TF 브로드캐스터 등록

- ICP 결과를 ROS2 TF 트리에 반영

- 명령어:

```bash
ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw base_link lidar2_link
```
- Launch 파일에 추가해 자동 브로드캐스트 → 모든 노드에서 공통 좌표계 사용 가능

3️⃣ fusion_node 실행

- TF2 버퍼에서 lidar1_link / lidar2_link 변환 자동 구독

- tf_buffer_->transform()으로 두 클라우드를 base_link 좌표계로 정렬

- 병합 후 /points_fused 발행 → 최종 정렬된 PointCloud 생성

🔑 핵심 포인트

- ICP는 한 번만 수행 → 결과는 TF에 영구 등록

- fusion_node는 변환값을 직접 계산하지 않고 TF로부터 받아서 사용

- 위치가 바뀌면 ICP만 다시 돌리고 TF 업데이트 → 코드 수정 필요 없음
