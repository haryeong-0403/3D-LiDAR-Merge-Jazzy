cmake_minimum_required(VERSION 3.8)
project(pointcloud_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)          
find_package(tf2_ros REQUIRED)      
find_package(tf2_eigen REQUIRED)   

include_directories(
  ${PCL_INCLUDE_DIRS}
)

# 실행 파일 추가 (이름을 pointcloud_fusion_node로 통일)
add_executable(pointcloud_fusion_node src/pointcloud_fusion.cpp)

ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  message_filters
  tf2          
  tf2_ros      
  tf2_eigen    
)

target_link_libraries(pointcloud_fusion_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pointcloud_fusion_node
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
