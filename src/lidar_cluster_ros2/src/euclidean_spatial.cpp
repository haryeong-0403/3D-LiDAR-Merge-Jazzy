// Non-grid (spatial) euclidean cluster filter for point cloud data
// based on https://github.com/autowarefoundation/autoware.universe/blob/main/perception/euclidean_cluster/lib/euclidean_cluster.cpp
// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>  // std::max

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/int32.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// ROS package
#include "lidar_cluster/marker.hpp"

#include "cluster_outline.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class EuclideanSpatial : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "minX") { minX = param.as_double(); }
      if (param.get_name() == "minY") { minY = param.as_double(); }
      if (param.get_name() == "minZ") { minZ = param.as_double(); }
      if (param.get_name() == "maxX") { maxX = param.as_double(); }
      if (param.get_name() == "maxY") { maxY = param.as_double(); }
      if (param.get_name() == "maxZ") { maxZ = param.as_double(); }

      if (param.get_name() == "points_in_topic")
      {
        points_in_topic = param.as_string();
        auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          points_in_topic, sensor_qos,
          std::bind(&EuclideanSpatial::lidar_callback, this, std::placeholders::_1));
      }
      if (param.get_name() == "points_out_topic")
      {
        points_out_topic = param.as_string();
        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
      }
      if (param.get_name() == "marker_out_topic")
      {
        marker_out_topic = param.as_string();
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
      }
      if (param.get_name() == "cluster_count_topic")
      {
        cluster_count_topic_ = param.as_string();
        pub_cluster_count_ = this->create_publisher<std_msgs::msg::Int32>(cluster_count_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "Republishing cluster_count_topic: %s", cluster_count_topic_.c_str());
      }

      if (param.get_name() == "verbose1") { verbose1 = param.as_bool(); }
      if (param.get_name() == "verbose2") { verbose2 = param.as_bool(); }
      if (param.get_name() == "tolerance") { tolerance_ = param.as_double(); }
      if (param.get_name() == "min_cluster_size") { min_cluster_size_ = param.as_int(); }
      if (param.get_name() == "max_cluster_size") { max_cluster_size_ = param.as_int(); }
      if (param.get_name() == "use_height") { use_height_ = param.as_bool(); }
    }
    return result;
  }

public:
  EuclideanSpatial() : Node("euclidean_spatial"), count_(0)
  {
    // ROI
    this->declare_parameter<float>("minX", minX);
    this->declare_parameter<float>("minY", minY);
    this->declare_parameter<float>("minZ", minZ);
    this->declare_parameter<float>("maxX", maxX);
    this->declare_parameter<float>("maxY", maxY);
    this->declare_parameter<float>("maxZ", maxZ);

    // Topics
    this->declare_parameter<std::string>("points_in_topic", "/lexus3/os_center/points");
    this->declare_parameter<std::string>("points_out_topic", "clustered_points");
    this->declare_parameter<std::string>("marker_out_topic", "clustered_marker");
    this->declare_parameter<std::string>("cluster_count_topic", "cluster_count");

    // Clustering & Log
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<float>("tolerance", tolerance_);
    this->declare_parameter<int>("min_cluster_size", min_cluster_size_);
    this->declare_parameter<int>("max_cluster_size", max_cluster_size_);
    this->declare_parameter<bool>("use_height", use_height_);

    // Get params
    this->get_parameter("minX", minX);
    this->get_parameter("minY", minY);
    this->get_parameter("minZ", minZ);
    this->get_parameter("maxX", maxX);
    this->get_parameter("maxY", maxY);
    this->get_parameter("maxZ", maxZ);

    this->get_parameter("points_in_topic", points_in_topic);
    this->get_parameter("points_out_topic", points_out_topic);
    this->get_parameter("marker_out_topic", marker_out_topic);
    this->get_parameter("cluster_count_topic", cluster_count_topic_);

    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("tolerance", tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("use_height", use_height_);

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

    RCLCPP_INFO(this->get_logger(), "EuclideanSpatial node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing cluster count to: '%s'", cluster_count_topic_.c_str());
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    visualization_msgs::msg::MarkerArray mark_array;

    // ROS -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *pointcloud);
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;

    const int original_size = pointcloud->width * pointcloud->height;

    // ROI Crop
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(pointcloud);
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.filter(*pointcloud);

    if (verbose1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size
        << " Reduced size: " << pointcloud->width * pointcloud->height);
    }

    // 2D/3D 선택
    if (!use_height_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pointcloud_2d_ptr->reserve(pointcloud->points.size());
      for (const auto &point : pointcloud->points)
      {
        pcl::PointXYZ p2d; p2d.x = point.x; p2d.y = point.y; p2d.z = 0.0;
        pointcloud_2d_ptr->push_back(p2d);
      }
      pointcloud_ptr = pointcloud_2d_ptr;
    }
    else
    {
      pointcloud_ptr = pointcloud;
    }

    // KdTree & Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud_ptr);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
    pcl_euclidean_cluster.setClusterTolerance(tolerance_);
    pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
    pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
    pcl_euclidean_cluster.setSearchMethod(tree);
    pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
    pcl_euclidean_cluster.extract(cluster_indices);

    // 클러스터 개수
    const int num_clusters = static_cast<int>(cluster_indices.size());
    if (verbose2) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Number of clusters: " << num_clusters);
    }

    // Publish cluster count
    if (pub_cluster_count_)
    {
      std_msgs::msg::Int32 cnt_msg;
      cnt_msg.data = num_clusters;
      pub_cluster_count_->publish(cnt_msg);
    }

    // 최대 클러스터 갱신 (고스트 마커 정리용)
    max_clust_reached = std::max(max_clust_reached, num_clusters);

    mark_array.markers.clear();

    // 출력 포인트(클러스터 ID를 intensity로)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    int intensity = 0;
    for (const auto &cluster : cluster_indices)
    {
      intensity++;
      double center_x = 0.0, center_y = 0.0;
      int count = 0;

      for (const auto &point_idx : cluster.indices)
      {
        pcl::PointXYZI pxyzi;
        pxyzi.x = pointcloud->points[point_idx].x;
        pxyzi.y = pointcloud->points[point_idx].y;
        pxyzi.z = pointcloud->points[point_idx].z;
        pxyzi.intensity = static_cast<float>(intensity);
        center_x += pxyzi.x;
        center_y += pxyzi.y;
        count++;
        cloud_cluster->points.push_back(pxyzi);
      }

      clusters.push_back(*cloud_cluster);
      clusters.back().width = cloud_cluster->points.size();
      clusters.back().height = 1;
      clusters.back().is_dense = false;

      if (count > 0) {
        center_x /= static_cast<double>(count);
        center_y /= static_cast<double>(count);
      }

      // 중심 마커 (ns/id 고정 규칙으로 중복 방지)
      visualization_msgs::msg::Marker center_marker;
      init_center_marker(center_marker, center_x, center_y, intensity);
      center_marker.ns = "center";
      center_marker.id = intensity;
      center_marker.header.frame_id = input_msg->header.frame_id;
      center_marker.header.stamp = this->now();
      mark_array.markers.push_back(center_marker);
    }

    const int num_of_clusters = static_cast<int>(clusters.size());

    // 고스트 마커 제거용 투명 마커 (center 네임스페이스용)
    for (int i = num_of_clusters + 1; i <= max_clust_reached; i++) {
      visualization_msgs::msg::Marker center_marker;
      init_center_marker(center_marker, 0.0, 0.0, i);
      center_marker.ns = "center";
      center_marker.id = i;
      center_marker.header.frame_id = input_msg->header.frame_id;
      center_marker.header.stamp = this->now();
      center_marker.color.a = 0.0;
      mark_array.markers.push_back(center_marker);
    }

    // 외곽선 (별도 구현에서 생성되는 마커들의 ns/id는 해당 구현에서 고유하도록 보장해야 함)
    cluster_outline.computeOutline(cloud_cluster, mark_array, 20, max_clust_reached, input_msg->header.frame_id);

    // PCL -> ROS & Publish
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_cluster, output_msg);
    output_msg.header.frame_id = input_msg->header.frame_id;
    output_msg.header.stamp = this->now();
    pub_lidar_->publish(output_msg);
    pub_marker_->publish(mark_array);
  }

  // pubs/subs
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_cluster_count_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  outline::ClusterOutline cluster_outline;

  // params/state
  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = 80.0,  maxY = +25.0, maxZ = -0.15;
  float tolerance_ = 0.02f;
  int min_cluster_size_ = 10, max_clust_reached = 0;
  int max_cluster_size_ = 500;
  bool use_height_ = false;
  bool verbose1 = false, verbose2 = false;

  std::string points_in_topic, points_out_topic, marker_out_topic;
  std::string cluster_count_topic_ = "cluster_count";
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EuclideanSpatial>());
  rclcpp::shutdown();
  return 0;
}
