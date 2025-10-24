cmake_minimum_required(VERSION 3.8)
project(lidar_cluster)

# -------------------------------------------------
# Build flags
# -------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-O2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------------------------
# Dependencies
# -------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)

# CPU 경로(PCL)만 사용 — GPU 모듈 제거
# filters(VoxelGrid, CropBox), segmentation(ExtractClusters)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation search)

# OpenMP 추가 (CPU 멀티코어 가속)
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  message(STATUS "OpenMP found, enabling parallel processing for CPU clustering.")
else()
  message(WARNING "OpenMP not found. Clustering performance will be limited to a single core.")
endif()

# -------------------------------------------------
# Include dirs
# -------------------------------------------------
set(INCLUDE_DIRS
  include
  ${PCL_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

# -------------------------------------------------
# ament deps list
# -------------------------------------------------
set(ament_dependencies
  pcl_conversions
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  rclcpp_components
)

# -------------------------------------------------
# Sources
# -------------------------------------------------
set(BENCHMARK_SOURCES
  src/benchmark.cpp
  include/benchmark.hpp
)

set(OUTLINE_SOURCES
  src/cluster_outline.cpp
  include/cluster_outline.hpp
)

# -------------------------------------------------
# Executables (기존 CPU 경로 그대로)
# -------------------------------------------------
add_executable(dbscan_spatial src/dbscan_spatial.cpp ${BENCHMARK_SOURCES} ${OUTLINE_SOURCES})
target_link_libraries(dbscan_spatial ${PCL_LIBRARIES})
ament_target_dependencies(dbscan_spatial ${ament_dependencies})
target_compile_features(dbscan_spatial PUBLIC c_std_99 cxx_std_17)

add_executable(dbscan_grid src/dbscan_grid.cpp)
target_link_libraries(dbscan_grid ${PCL_LIBRARIES})
ament_target_dependencies(dbscan_grid ${ament_dependencies})
target_compile_features(dbscan_grid PUBLIC c_std_99 cxx_std_17)

add_executable(dblane_spatial src/dblane_spatial.cpp)
target_link_libraries(dblane_spatial ${PCL_LIBRARIES})
ament_target_dependencies(dblane_spatial ${ament_dependencies})
target_compile_features(dblane_spatial PUBLIC c_std_99 cxx_std_17)

add_executable(dblane_f1s src/dblane_f1s.cpp)
target_link_libraries(dblane_f1s ${PCL_LIBRARIES})
ament_target_dependencies(dblane_f1s ${ament_dependencies})
target_compile_features(dblane_f1s PUBLIC c_std_99 cxx_std_17)

add_executable(euclidean_grid src/euclidean_grid.cpp ${BENCHMARK_SOURCES} ${OUTLINE_SOURCES})
target_link_libraries(euclidean_grid ${PCL_LIBRARIES})
ament_target_dependencies(euclidean_grid ${ament_dependencies})
target_compile_features(euclidean_grid PUBLIC c_std_99 cxx_std_17)

# -------------------------------------------------
# Composable node (기존 그대로)
# -------------------------------------------------
add_library(euclidean_grid_composable SHARED src/euclidean_grid_composable.cpp ${BENCHMARK_SOURCES} ${OUTLINE_SOURCES})
target_include_directories(euclidean_grid_composable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(euclidean_grid_composable ${PCL_LIBRARIES})
ament_target_dependencies(euclidean_grid_composable ${ament_dependencies})
rclcpp_components_register_nodes(euclidean_grid_composable
  PLUGIN "cluster::EuclideanGridCore"
  EXECUTABLE euclidean_grid_composable)

# -------------------------------------------------
# ★ OpenMP 가속 Euclidean Clustering 실행파일 (CPU)
# -------------------------------------------------
# 수정: C++ 코드에서 ${OUTLINE_SOURCES}를 사용하지 않으므로 제거
add_executable(euclidean_spatial src/euclidean_spatial.cpp)
target_include_directories(euclidean_spatial PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(euclidean_spatial
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)
ament_target_dependencies(euclidean_spatial ${ament_dependencies})
target_compile_features(euclidean_spatial PUBLIC c_std_99 cxx_std_17)

# -------------------------------------------------
# Install & Export
# -------------------------------------------------
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_euclidean_grid_composable)

install(TARGETS euclidean_grid_composable
        EXPORT export_euclidean_grid_composable
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
  dbscan_spatial
  dbscan_grid
  dblane_spatial
  euclidean_spatial # <-- 설치 목록에 포함 확인
  dblane_f1s
  euclidean_grid
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# -------------------------------------------------
# Tests
# -------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()