// Non-grid (spatial) Euclidean cluster filter for point cloud data (CPU optimized)
// - Replaced PCL GPU with PCL CPU implementation
// - Added VoxelGrid downsampling for massive performance boost
// - Uses OpenMP-enabled KdTree and EuclideanClusterExtraction for multi-core processing

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/int32.hpp"

// PCL CPU
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h> // 다운샘플링 (성능 향상)
#include <pcl/search/kdtree.h> // CPU 검색 (OpenMP 지원)
#include <pcl/segmentation/extract_clusters.h> // CPU 클러스터링 (OpenMP 지원)

// ROS package
#include "lidar_cluster/marker.hpp"
// #include "cluster_outline.hpp" // 원본 코드에서 사용되지 않아 주석 처리

using namespace std::chrono_literals;
using std::placeholders::_1;

class EuclideanSpatial : public rclcpp::Node
{
public:
  EuclideanSpatial() : Node("LiDAR_Object_Detection"), count_(0)
  {
    // Declare parameters
    this->declare_parameter<float>("minX", minX);
    this->declare_parameter<float>("minY", minY);
    this->declare_parameter<float>("minZ", minZ);
    this->declare_parameter<float>("maxX", maxX);
    this->declare_parameter<float>("maxY", maxY);
    this->declare_parameter<float>("maxZ", maxZ);
    this->declare_parameter<std::string>("points_in_topic", "/merged_points");
    this->declare_parameter<std::string>("points_out_topic", "clustered_points");
    this->declare_parameter<std::string>("marker_out_topic", "clustered_marker");
    this->declare_parameter<std::string>("cluster_count_topic", "cluster_count");
    this->declare_parameter<float>("tolerance", tolerance_);
    this->declare_parameter<int>("min_cluster_size", min_cluster_size_);
    this->declare_parameter<int>("max_cluster_size", max_cluster_size_);
    this->declare_parameter<bool>("use_height", use_height_);
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);

    // --- 성능 향상을 위한 VoxelGrid 파라미터 ---
    this->declare_parameter<float>("voxel_leaf_size", voxel_leaf_size_);

    // Get parameters
    this->get_parameter("points_in_topic", points_in_topic);
    this->get_parameter("points_out_topic", points_out_topic);
    this->get_parameter("marker_out_topic", marker_out_topic);
    this->get_parameter("cluster_count_topic", cluster_count_topic_);
    this->get_parameter("tolerance", tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("use_height", use_height_);
    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

    // Publishers
    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    pub_cluster_count_ = this->create_publisher<std_msgs::msg::Int32>(cluster_count_topic_, 10);

    // Subscriber
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_in_topic, sensor_qos,
      std::bind(&EuclideanSpatial::lidar_callback, this, std::placeholders::_1));

    // Parameter callback
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EuclideanSpatial::parametersCallback, this, std::placeholders::_1));

    // --- PCL 객체 멤버 변수로 초기화 (콜백마다 생성 방지) ---
    tree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    vg_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    ec_.setSearchMethod(tree_); // KdTree를 클러스터링 객체에 연결

    RCLCPP_INFO(this->get_logger(), "[CPU] EuclideanSpatial node started.");
    RCLCPP_INFO(this->get_logger(), "Voxel Leaf Size: %.2f m", voxel_leaf_size_);
  }

private:
  // Parameter update callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : params)
    {
      if (param.get_name() == "tolerance") tolerance_ = param.as_double();
      if (param.get_name() == "min_cluster_size") min_cluster_size_ = param.as_int();
      if (param.get_name() == "max_cluster_size") max_cluster_size_ = param.as_int();
      if (param.get_name() == "voxel_leaf_size")
      {
        voxel_leaf_size_ = param.as_double();
        vg_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "Set Voxel Leaf Size: %.2f m", voxel_leaf_size_);
      }
    }
    return result;
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    visualization_msgs::msg::MarkerArray mark_array;

    // ROS → PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cpu(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *cloud_cpu);

    // ROI Crop
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(cloud_cpu);
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.filter(*cloud_cpu);

    if (cloud_cpu->empty())
      return;

    // --- 1. VoxelGrid 다운샘플링 (성능 향상) ---
    // 클러스터링할 포인트 수를 대폭 줄여 속도를 높입니다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg_filter_.setInputCloud(cloud_cpu);
    vg_filter_.filter(*cloud_filtered);

    if (cloud_filtered->empty())
      return;

    // z=0 for 2D clustering
    if (!use_height_)
    {
      // 다운샘플링된 클라우드의 z값을 0으로 만듭니다.
      for (auto &p : cloud_filtered->points)
        p.z = 0.0f;
    }

    // ---- 2. CPU Stage (OpenMP 병렬 처리) ----
    // PCL이 OpenMP로 컴파일되었다면 이 부분은 자동으로 멀티코어를 사용합니다.

    // KdTree 생성 및 입력 (멤버 변수 사용)
    tree_->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    
    // 클러스터링 객체 설정 (멤버 변수 사용)
    ec_.setClusterTolerance(tolerance_);
    ec_.setMinClusterSize(min_cluster_size_);
    ec_.setMaxClusterSize(max_cluster_size_);
    ec_.setInputCloud(cloud_filtered);
    ec_.extract(cluster_indices); // 클러스터링 실행

    // ------------------------------------------

    int num_clusters = static_cast<int>(cluster_indices.size());
    if (verbose2)
      RCLCPP_INFO_STREAM(this->get_logger(), "[CPU] Cluster count: " << num_clusters);

    // Publish cluster count
    std_msgs::msg::Int32 count_msg;
    count_msg.data = num_clusters;
    pub_cluster_count_->publish(count_msg);

    // Output combined cloud (intensity = cluster ID)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    int cluster_id = 0;

    for (const auto &c : cluster_indices)
    {
      cluster_id++;
      double cx = 0.0, cy = 0.0;
      int cnt = 0;

      for (int idx : c.indices)
      {
        pcl::PointXYZI p;
        // 중요: cloud_cpu가 아닌 cloud_filtered에서 포인트를 가져옵니다.
        p.x = cloud_filtered->points[idx].x;
        p.y = cloud_filtered->points[idx].y;
        p.z = cloud_filtered->points[idx].z;
        p.intensity = static_cast<float>(cluster_id);
        cx += p.x; cy += p.y;
        cloud_out->points.push_back(p);
        cnt++;
      }

      if (cnt > 0)
      {
        cx /= cnt; cy /= cnt;
        visualization_msgs::msg::Marker center_marker;
        init_center_marker(center_marker, cx, cy, cluster_id);
        center_marker.ns = "center";
        center_marker.id = cluster_id;
        center_marker.header.frame_id = input_msg->header.frame_id;
        center_marker.header.stamp = this->now();
        mark_array.markers.push_back(center_marker);
      }
    }

    pcl::toROSMsg(*cloud_out, output_msg_);
    output_msg_.header.frame_id = input_msg->header.frame_id;
    output_msg_.header.stamp = this->now();

    pub_lidar_->publish(output_msg_);
    pub_marker_->publish(mark_array);
  }

  // --- Publishers/Subscribers ---
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cluster_count_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  sensor_msgs::msg::PointCloud2 output_msg_;

  // --- PCL 객체 (멤버 변수) ---
  pcl::VoxelGrid<pcl::PointXYZ> vg_filter_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;

  // Params
  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = 80.0, maxY = +25.0, maxZ = 2.0;
  float tolerance_ = 0.2f;
  int min_cluster_size_ = 5, max_cluster_size_ = 3000;
  bool use_height_ = false;
  bool verbose1 = false, verbose2 = true;
  float voxel_leaf_size_ = 0.1f; // 10cm 복셀 (조정 가능)

  std::string points_in_topic, points_out_topic, marker_out_topic, cluster_count_topic_;
  size_t count_;
  // outline::ClusterOutline cluster_outline; // 원본 코드에서 사용되지 않아 제거
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EuclideanSpatial>());
  rclcpp::shutdown();
  return 0;
}