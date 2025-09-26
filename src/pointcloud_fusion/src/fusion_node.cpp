#include <iostream>
#include <memory>
#include <string>
#include <functional>                 
#include <tf2/exceptions.h>    

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> // ROS 2에서 PointCloud2 변환용

// Message Filters
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// 사용할 PCL 포인트 클라우드 타입 정의
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
// 사용할 ROS 메시지 타입 정의
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode() : Node("pointcloud_fusion_node")
  {
    // 파라미터 선언 및 초기화 (변환할 목표 좌표계)
    this->declare_parameter<std::string>("target_frame", "base_link");
    target_frame_ = this->get_parameter("target_frame").as_string();

    // TF2 리스너 및 버퍼 초기화
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 병합된 포인트 클라우드를 발행할 퍼블리셔
    publisher_ = this->create_publisher<PointCloud2>("/merged_points", 10);

    // 두 LiDAR 토픽을 구독할 서브스크라이버
    // 토픽 이름은 launch 파일에서 remap을 통해 지정할 예정
    sub1_.subscribe(this, "/lidar1/points");
    sub2_.subscribe(this, "/lidar2/points");

    // ApproximateTime 정책을 사용한 동기화 설정
    sync_ = std::make_shared<Sync>(SyncPolicy(10), sub1_, sub2_);
    sync_->registerCallback(std::bind(&PointCloudFusionNode::topic_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "PointCloud fusion node has been started.");
    RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
  }

private:
  // message_filters 타입 정의
  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  // 콜백 함수
  void topic_callback(const PointCloud2::ConstSharedPtr &cloud1_msg, const PointCloud2::ConstSharedPtr &cloud2_msg)
  {
    PointCloud2 cloud1_transformed, cloud2_transformed;

    try
    {
      // ROS 2에서는 tf2_sensor_msgs::do_transform 함수 대신 버퍼의 transform 함수를 사용
      tf_buffer_->transform(*cloud1_msg, cloud1_transformed, target_frame_);
      tf_buffer_->transform(*cloud2_msg, cloud2_transformed, target_frame_);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform cloud: %s", ex.what());
      return;
    }

    // PCL 타입으로 변환하여 병합
    PointCloud pcl_cloud1, pcl_cloud2;
    pcl::fromROSMsg(cloud1_transformed, pcl_cloud1);
    pcl::fromROSMsg(cloud2_transformed, pcl_cloud2);

    PointCloud merged_pcl = pcl_cloud1 + pcl_cloud2;

    // 다시 ROS 메시지 타입으로 변환하여 발행
    PointCloud2 merged_msg;
    pcl::toROSMsg(merged_pcl, merged_msg);
    merged_msg.header.stamp = this->get_clock()->now(); // 최신 시간으로 스탬프
    merged_msg.header.frame_id = target_frame_; // 목표 좌표계로 프레임 ID 설정

    publisher_->publish(merged_msg);
  }

  // 멤버 변수
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  message_filters::Subscriber<PointCloud2> sub1_;
  message_filters::Subscriber<PointCloud2> sub2_;
  std::shared_ptr<Sync> sync_;
  
  std::string target_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFusionNode>());
  rclcpp::shutdown();
  return 0;
}