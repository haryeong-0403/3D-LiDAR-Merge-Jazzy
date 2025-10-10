#include <iostream>
#include <memory>
#include <string>
#include <cmath>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct OrientedBox2D {
  // base_link 좌표계 기준
  float cx{0.0f};
  float cy{0.0f};
  float length{1.128f};   // x 방향 총 길이 (m)
  float width{0.798f};    // y 방향 총 폭 (m)
  float yaw{0.0f};        // rad, 상향(+Z) 회전 (RViz의 파랑축 기준)
  float margin{0.01f};    // 여유 확장 (m)
  bool  enabled{true};
  // z 필터(선택): 로봇 높이 범위 지정해서 그 사이만 제거
  bool  use_z{false};
  float z_min{-0.10f};
  float z_max{0.60f};
};

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode()
  : Node("pointcloud_fusion_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // === 파라미터 선언 ===
    declare_parameter<std::string>("output_topic", "/merged_points");
    declare_parameter<std::string>("fixed_frame", "base_link"); // 병합 및 필터 기준 프레임
    // 메인 박스(로봇 본체)
    declare_parameter<bool>("self_box.enabled", true);
    declare_parameter<double>("self_box.cx", 0.0);
    declare_parameter<double>("self_box.cy", 0.0);
    declare_parameter<double>("self_box.length", 1.128);
    declare_parameter<double>("self_box.width", 0.798);
    declare_parameter<double>("self_box.yaw_deg", 0.0);
    declare_parameter<double>("self_box.margin", 0.02);
    declare_parameter<bool>("self_box.use_z", false);
    declare_parameter<double>("self_box.z_min", -0.10);
    declare_parameter<double>("self_box.z_max", 0.60);
    // 보조 박스(배터리함/폴 등 필요시)
    declare_parameter<bool>("extra_box.enabled", false);
    declare_parameter<double>("extra_box.cx", 0.0);
    declare_parameter<double>("extra_box.cy", 0.0);
    declare_parameter<double>("extra_box.length", 0.30);
    declare_parameter<double>("extra_box.width", 0.25);
    declare_parameter<double>("extra_box.yaw_deg", 0.0);
    declare_parameter<double>("extra_box.margin", 0.0);
    declare_parameter<bool>("extra_box.use_z", false);
    declare_parameter<double>("extra_box.z_min", -0.10);
    declare_parameter<double>("extra_box.z_max", 0.60);
    // 반경 필터(선택)
    declare_parameter<bool>("radius_filter.enabled", false);
    declare_parameter<double>("radius_filter.r", 0.60);
    declare_parameter<double>("radius_filter.cx", 0.0);
    declare_parameter<double>("radius_filter.cy", 0.0);

    // === 파라미터 로딩 ===
    std::string out_topic, fixed_frame;
    get_parameter("output_topic", out_topic);
    get_parameter("fixed_frame", fixed_frame);
    fixed_frame_ = fixed_frame;

    self_box_ = loadBoxParams("self_box");
    extra_box_ = loadBoxParams("extra_box");
    use_extra_box_ = get_parameter("extra_box.enabled").as_bool();

    use_radius_filter_ = get_parameter("radius_filter.enabled").as_bool();
    radius_r_ = static_cast<float>(get_parameter("radius_filter.r").as_double());
    radius_cx_ = static_cast<float>(get_parameter("radius_filter.cx").as_double());
    radius_cy_ = static_cast<float>(get_parameter("radius_filter.cy").as_double());

    publisher_ = this->create_publisher<PointCloud2>(out_topic, rclcpp::SensorDataQoS());

    // LiDAR 2개 구독 (드라이버 토픽)
    sub1_.subscribe(this, "/lidar1/cloud_unstructured_fullframe");
    sub2_.subscribe(this, "/lidar2/cloud_unstructured_fullframe");

    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub1_, sub2_);
    sync_->registerCallback(
        std::bind(&PointCloudFusionNode::topic_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "PointCloud fusion node with self-filter started.");
    RCLCPP_INFO(get_logger(), "Output: %s | Fixed frame: %s", out_topic.c_str(), fixed_frame_.c_str());
  }

private:
  // === 타입 ===
  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  // === 콜백 ===
  void topic_callback(const PointCloud2::ConstSharedPtr &cloud1_msg,
                      const PointCloud2::ConstSharedPtr &cloud2_msg)
  {
    try
    {
      // 둘 다 fixed_frame_ (기본 base_link)로 변환
      const auto tf_lidar1 =
          tf_buffer_.lookupTransform(fixed_frame_, cloud1_msg->header.frame_id, tf2::TimePointZero);
      const auto tf_lidar2 =
          tf_buffer_.lookupTransform(fixed_frame_, cloud2_msg->header.frame_id, tf2::TimePointZero);

      PointCloud pcl_cloud1, pcl_cloud2;
      pcl::fromROSMsg(*cloud1_msg, pcl_cloud1);
      pcl::fromROSMsg(*cloud2_msg, pcl_cloud2);

      Eigen::Matrix4f T1 = tf2::transformToEigen(tf_lidar1.transform).matrix().cast<float>();
      Eigen::Matrix4f T2 = tf2::transformToEigen(tf_lidar2.transform).matrix().cast<float>();

      PointCloud pcl_cloud1_tf, pcl_cloud2_tf;
      pcl::transformPointCloud(pcl_cloud1, pcl_cloud1_tf, T1);
      pcl::transformPointCloud(pcl_cloud2, pcl_cloud2_tf, T2);
      
      // The fix is on the next line
      PointCloud merged_pcl = pcl_cloud1_tf + pcl_cloud2_tf;

      // === 여기서 self-filter 적용 ===
      if (use_radius_filter_ && radius_r_ > 0.0f) {
        removePointsWithinRadius(merged_pcl, radius_r_, radius_cx_, radius_cy_);
      }
      removePointsInOrientedBox2D(merged_pcl, self_box_);
      if (use_extra_box_) removePointsInOrientedBox2D(merged_pcl, extra_box_);

      // ★★ 중요: 필터로 points 개수가 바뀌었으니 메타데이터를 "최종적으로" 재설정 ★★
      merged_pcl.is_dense = false;
      merged_pcl.width  = static_cast<uint32_t>(merged_pcl.points.size());
      merged_pcl.height = 1;

      // ROS 메시지로 변환
      PointCloud2 merged_msg;
      pcl::toROSMsg(merged_pcl, merged_msg);
      merged_msg.header.stamp = this->get_clock()->now();
      merged_msg.header.frame_id = fixed_frame_;
      publisher_->publish(merged_msg);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  // === 박스 파라미터 로더 ===
  OrientedBox2D loadBoxParams(const std::string& ns) {
    OrientedBox2D b;
    b.enabled = get_parameter(ns + ".enabled").as_bool();
    b.cx = static_cast<float>(get_parameter(ns + ".cx").as_double());
    b.cy = static_cast<float>(get_parameter(ns + ".cy").as_double());
    b.length = static_cast<float>(get_parameter(ns + ".length").as_double());
    b.width  = static_cast<float>(get_parameter(ns + ".width").as_double());
    const double yaw_deg = get_parameter(ns + ".yaw_deg").as_double();
    b.yaw = static_cast<float>(yaw_deg * M_PI / 180.0);
    b.margin = static_cast<float>(get_parameter(ns + ".margin").as_double());
    b.use_z = get_parameter(ns + ".use_z").as_bool();
    b.z_min = static_cast<float>(get_parameter(ns + ".z_min").as_double());
    b.z_max = static_cast<float>(get_parameter(ns + ".z_max").as_double());
    return b;
  }

  // === 회전 사각형(2D) 내부 제거 ===
  void removePointsInOrientedBox2D(PointCloud& cloud, const OrientedBox2D& box)
  {
    if (!box.enabled) return;

    const float hl = 0.5f * (box.length + 2.f * box.margin);
    const float hw = 0.5f * (box.width  + 2.f * box.margin);

    // -yaw 회전(포인트를 박스 좌표계로)
    const float c = std::cos(-box.yaw);
    const float s = std::sin(-box.yaw);

    PointCloud filtered;
    filtered.points.reserve(cloud.points.size());

    for (const auto& p : cloud.points) {
      // (선택) z 범위 내에서만 self-filter 적용
      if (box.use_z) {
        if (p.z < box.z_min || p.z > box.z_max) {
          filtered.points.push_back(p);
          continue;
        }
      }
      // 평행이동 → 회전
      const float tx = p.x - box.cx;
      const float ty = p.y - box.cy;
      const float rx = c * tx - s * ty;
      const float ry = s * tx + c * ty;

      const bool inside = (std::fabs(rx) <= hl) && (std::fabs(ry) <= hw);
      if (!inside) filtered.points.push_back(p);
    }
    cloud.points.swap(filtered.points);
  }

  // === 반경 내부 제거 ===
  void removePointsWithinRadius(PointCloud& cloud, float r, float cx, float cy)
  {
    PointCloud filtered;
    filtered.points.reserve(cloud.points.size());
    const float r2 = r * r;
    for (const auto& p : cloud.points) {
      const float dx = p.x - cx, dy = p.y - cy;
      if (dx*dx + dy*dy >= r2) filtered.points.push_back(p);
    }
    cloud.points.swap(filtered.points);
  }

  // === 멤버들 ===
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  message_filters::Subscriber<PointCloud2> sub1_;
  message_filters::Subscriber<PointCloud2> sub2_;
  std::shared_ptr<Sync> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string fixed_frame_{"base_link"};

  OrientedBox2D self_box_;
  OrientedBox2D extra_box_;
  bool use_extra_box_{false};

  bool  use_radius_filter_{false};
  float radius_r_{0.0f};
  float radius_cx_{0.0f};
  float radius_cy_{0.0f};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}