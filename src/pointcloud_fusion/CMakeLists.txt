cmake_minimum_required(VERSION 3.10)
project(pointcloud_fusion)

# C++ 표준(필요하면 유지/조정)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Qhull 찾기 & 호환 타깃 정의 (pcl_ros가 기대하는 QHULL::QHULL 보정) ---
find_package(Qhull QUIET CONFIG)
if(NOT Qhull_FOUND)
  find_package(Qhull QUIET)  # module mode fallback
endif()

# Ubuntu 22.04 등에서 종종 Qhull::qhull_r만 있고 QHULL::QHULL은 없는 경우가 있음
if(TARGET Qhull::qhull_r AND NOT TARGET QHULL::QHULL)
  add_library(QHULL::QHULL INTERFACE IMPORTED)
  target_link_libraries(QHULL::QHULL INTERFACE Qhull::qhull_r)
endif()
# --------------------------------------------------------------------------

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# 실행 파일 추가
add_executable(fusion_node src/fusion_node.cpp)
add_executable(pcd_saver_node src/pcd_saver_node.cpp)
add_executable(transform_pcd_node src/transform_pcd_node.cpp)


target_include_directories(fusion_node BEFORE PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)

# ament 종속성
ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_sensor_msgs
  message_filters
)
ament_target_dependencies(pcd_saver_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# PCL 라이브러리 링크
target_link_libraries(fusion_node ${PCL_LIBRARIES})
target_link_libraries(pcd_saver_node ${PCL_LIBRARIES})
target_link_libraries(transform_pcd_node ${PCL_LIBRARIES})


# 설치 (모든 실행 파일을 한 번에 설치하도록 통합)
install(TARGETS
  fusion_node
  pcd_saver_node
  transform_pcd_node # <-- 여기에 함께 추가
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()