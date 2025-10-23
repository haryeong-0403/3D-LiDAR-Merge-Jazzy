#ifndef MULTIPLE_OBJECT_TRACKING_LIDAR_HPP
#define MULTIPLE_OBJECT_TRACKING_LIDAR_HPP

#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"  // ★ MarkerArray 추가

#include "opencv2/core.hpp"
#include "opencv2/video/tracking.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

namespace multiple_object_tracking_lidar
{

class MultipleObjectTrackingLidar : public rclcpp::Node {
public:
  explicit MultipleObjectTrackingLidar(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  MultipleObjectTrackingLidar(
      const std::string& name_space,
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ---- ROS I/F ----
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;

  // ★ MarkerArray 퍼블리셔로 교체 (이름도 관례상 marker_pub_)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr objID_pub;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster1;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster2;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster3;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster4;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster5;

  // ---- 상태/파라미터 ----
  std::vector<geometry_msgs::msg::Point> prevClusterCenters;
  std::vector<int> objID;    // KF 데이터 연관 결과
  bool firstFrame = true;

  rclcpp::Clock::SharedPtr clock_;
  std::string frame_id{"base_scan"};
  std::string filtered_cloud{"/merged_points"};

  // === 입력 메시지 헤더 캐시(퍼블리시 헤더를 입력과 동일하게 맞추기 위함) ===
  std::string last_input_frame_id_;
  builtin_interfaces::msg::Time last_input_stamp_;
  
  // ★ 전역이던 걸 클래스 멤버로 이동 (내 .cpp는 멤버를 기대함)
  int stateDim{4};  // [x, y, v_x, v_y]
  int measDim{2};   // [z_x, z_y]
  int ctrlDim{0};

  // ★ 6개의 KalmanFilter를 멤버로 보유 (내 .cpp와 일치)
  cv::KalmanFilter KF0{4, 2, 0};
  cv::KalmanFilter KF1{4, 2, 0};
  cv::KalmanFilter KF2{4, 2, 0};
  cv::KalmanFilter KF3{4, 2, 0};
  cv::KalmanFilter KF4{4, 2, 0};
  cv::KalmanFilter KF5{4, 2, 0};

  // ---- 메서드 ----
  double euclidean_distance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2);
  std::pair<int, int> findIndexOfMin(std::vector<std::vector<float>> distMat);
  void kft(const std_msgs::msg::Float32MultiArray ccs);
  void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);

  // ★ Jazzy에서 권장되는 콜백 시그니처 (ConstSharedPtr)
  void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);
};

} // namespace multiple_object_tracking_lidar

#endif // MULTIPLE_OBJECT_TRACKING_LIDAR_HPP
