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

// TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode() : Node("pointcloud_fusion_node"),
                           tf_buffer_(this->get_clock()),
                           tf_listener_(tf_buffer_)
  {
    publisher_ = this->create_publisher<PointCloud2>("/merged_points", 10);

    // LiDAR 2개 구독 (드라이버에서 publish한 토픽 이름에 맞춤)
    sub1_.subscribe(this, "/lidar1/cloud_unstructured_fullframe");
    sub2_.subscribe(this, "/lidar2/cloud_unstructured_fullframe");

    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub1_, sub2_);
    sync_->registerCallback(
        std::bind(&PointCloudFusionNode::topic_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "PointCloud fusion node started (TF-based).");
  }

private:
  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  void topic_callback(const PointCloud2::ConstSharedPtr &cloud1_msg,
                      const PointCloud2::ConstSharedPtr &cloud2_msg)
  {
    try
    {
      // TF에서 lidar1_link → base_link 변환
      geometry_msgs::msg::TransformStamped tf_lidar1 =
          tf_buffer_.lookupTransform("base_link", cloud1_msg->header.frame_id, tf2::TimePointZero);

      // TF에서 lidar2_link → base_link 변환
      geometry_msgs::msg::TransformStamped tf_lidar2 =
          tf_buffer_.lookupTransform("base_link", cloud2_msg->header.frame_id, tf2::TimePointZero);

      // ROS → PCL 변환
      PointCloud pcl_cloud1, pcl_cloud2;
      pcl::fromROSMsg(*cloud1_msg, pcl_cloud1);
      pcl::fromROSMsg(*cloud2_msg, pcl_cloud2);

      // 변환 적용
      Eigen::Matrix4f T1 = tf2::transformToEigen(tf_lidar1.transform).matrix().cast<float>();
      Eigen::Matrix4f T2 = tf2::transformToEigen(tf_lidar2.transform).matrix().cast<float>();

      PointCloud pcl_cloud1_tf, pcl_cloud2_tf;
      pcl::transformPointCloud(pcl_cloud1, pcl_cloud1_tf, T1);
      pcl::transformPointCloud(pcl_cloud2, pcl_cloud2_tf, T2);

      // 병합
      PointCloud merged_pcl = pcl_cloud1_tf + pcl_cloud2_tf;

      // ROS 메시지로 변환
      PointCloud2 merged_msg;
      pcl::toROSMsg(merged_pcl, merged_msg);
      merged_msg.header.stamp = this->get_clock()->now();
      merged_msg.header.frame_id = "base_link";

      publisher_->publish(merged_msg);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  message_filters::Subscriber<PointCloud2> sub1_;
  message_filters::Subscriber<PointCloud2> sub2_;
  std::shared_ptr<Sync> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}
