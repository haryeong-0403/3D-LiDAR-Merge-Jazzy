//ICP code

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PcdSaverNode : public rclcpp::Node
{
public:
  PcdSaverNode() : Node("pcd_saver_node")
  {
    // 파라미터로 저장할 파일 이름을 받습니다.
    this->declare_parameter<std::string>("output_file", "saved_cloud.pcd");
    output_file_ = this->get_parameter("output_file").as_string();

    // 'input' 토픽으로 들어오는 PointCloud2 메시지를 구독합니다.
    // std::bind를 사용하여 콜백 함수를 등록하고, qos(10)은 신뢰성 있는 연결을 보장합니다.
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", 10, std::bind(&PcdSaverNode::topic_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "PCD saver node started. Waiting for a message on topic 'input' to save to '%s'", output_file_.c_str());
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received a point cloud message!");

    // ROS 메시지를 PCL 포인트 클라우드 타입으로 변환합니다.
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // PCL의 PCDWriter를 사용하여 파일로 저장합니다.
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(output_file_, cloud, false); // false는 바이너리 형식이 아님을 의미

    RCLCPP_INFO(this->get_logger(), "Successfully saved point cloud to %s", output_file_.c_str());
    
    // 한 번 저장 후 노드를 종료합니다.
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::string output_file_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdSaverNode>());
  rclcpp::shutdown();
  return 0;
}