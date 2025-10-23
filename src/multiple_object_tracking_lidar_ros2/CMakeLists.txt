cmake_minimum_required(VERSION 3.12)
project(multiple_object_tracking_lidar)

# C++ 표준 (ROS 2 권장)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_C_STANDARD 99)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

# OpenCV 추가
find_package(OpenCV REQUIRED COMPONENTS core video)

# 선택(이미지/디버그 시 사용): 없으면 조용히 패스
find_package(cv_bridge QUIET)

# === Targets ===
add_library(${PROJECT_NAME} SHARED
  src/multiple_object_tracking_lidar.cpp
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# include 경로
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/multiple_object_tracking_lidar>
  $<INSTALL_INTERFACE:include>
)

# ament dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

# 선택 의존(cv_bridge)이 있으면 연결
if(cv_bridge_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${cv_bridge_INCLUDE_DIRS})
  ament_target_dependencies(${PROJECT_NAME} cv_bridge)
endif()

# 구성요소 등록(컴포넌트 사용 시)
rclcpp_components_register_nodes(${PROJECT_NAME}
  "multiple_object_tracking_lidar::MultipleObjectTrackingLidar"
)
set(node_plugins
  "${node_plugins}multiple_object_tracking_lidar::MultipleObjectTrackingLidar;$<TARGET_FILE:${PROJECT_NAME}>\n"
)

# 실행 노드
add_executable(${PROJECT_NAME}_node
  src/multiple_object_tracking_lidar_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}                 
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)

# === Install ===
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# launch 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === Tests / Lint ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
