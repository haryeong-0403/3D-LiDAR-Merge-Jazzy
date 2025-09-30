#include <iostream>
#include <memory>
#include <string>

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

// 사용할 PCL 포인트 클라우드 타입 정의
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode() : Node("pointcloud_fusion_node")
  {
    // 퍼블리셔
    publisher_ = this->create_publisher<PointCloud2>("/merged_points", 10);

    // LiDAR 2개 구독
    sub1_.subscribe(this, "/lidar1/points");
    sub2_.subscribe(this, "/lidar2/points");

    // ApproximateTime 동기화
    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub1_, sub2_);
    sync_->registerCallback(
      std::bind(&PointCloudFusionNode::topic_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PointCloud fusion node started with static transforms.");
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  void topic_callback(const PointCloud2::ConstSharedPtr &cloud1_msg,
                      const PointCloud2::ConstSharedPtr &cloud2_msg)
  {
    // ROS → PCL 변환
    PointCloud pcl_cloud1, pcl_cloud2;
    pcl::fromROSMsg(*cloud1_msg, pcl_cloud1);
    pcl::fromROSMsg(*cloud2_msg, pcl_cloud2);

    // 고정된 변환 행렬 정의 (예시: lidar1은 base_link 원점, lidar2는 y=0.7m 옆)
    Eigen::Matrix4f T_lidar1_to_base = Eigen::Matrix4f::Identity();
    T_lidar1_to_base(0,3) = 0.0;  // x offset
    T_lidar1_to_base(1,3) = 0.0;  // y offset
    T_lidar1_to_base(2,3) = 0.0;  // z offset

    Eigen::Matrix4f T_lidar2_to_base = Eigen::Matrix4f::Identity();
    T_lidar2_to_base(0,3) = 0.0;
    T_lidar2_to_base(1,3) = 0.7;   // y 방향으로 0.7m 떨어져 있다고 가정
    T_lidar2_to_base(2,3) = 0.0;

    // 변환 적용
    PointCloud pcl_cloud1_tf, pcl_cloud2_tf;
    pcl::transformPointCloud(pcl_cloud1, pcl_cloud1_tf, T_lidar1_to_base);
    pcl::transformPointCloud(pcl_cloud2, pcl_cloud2_tf, T_lidar2_to_base);

    // 병합
    PointCloud merged_pcl = pcl_cloud1_tf + pcl_cloud2_tf;

    // 다시 ROS 메시지로 변환
    PointCloud2 merged_msg;
    pcl::toROSMsg(merged_pcl, merged_msg);
    merged_msg.header.stamp = this->get_clock()->now();
    merged_msg.header.frame_id = "base_link"; // 항상 base_link 기준으로 publish

    publisher_->publish(merged_msg);
  }

  // 멤버 변수
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  message_filters::Subscriber<PointCloud2> sub1_;
  message_filters::Subscriber<PointCloud2> sub2_;
  std::shared_ptr<Sync> sync_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
