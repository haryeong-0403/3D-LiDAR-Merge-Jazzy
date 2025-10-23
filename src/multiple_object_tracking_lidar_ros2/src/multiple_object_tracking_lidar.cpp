#include "multiple_object_tracking_lidar.hpp"
#include <rclcpp/qos.hpp>
#include <cmath>

namespace multiple_object_tracking_lidar
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("multiple_object_tracking_lidar");

MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(
  const rclcpp::NodeOptions& options
): MultipleObjectTrackingLidar("", options)
{}

MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("MultipleObjectTrackingLidar", name_space, options)
{
  RCLCPP_INFO(this->get_logger(),"MultipleObjectTrackingLidar init complete!");

  this->declare_parameter("stateDim", 4);
  this->declare_parameter("measDim", 2);
  this->declare_parameter("ctrlDim", 0);
  this->declare_parameter("frame_id", "base_scan");
  this->declare_parameter("filtered_cloud", "/merged_points");

  this->get_parameter("stateDim", stateDim);
  this->get_parameter("measDim", measDim);
  this->get_parameter("ctrlDim", ctrlDim);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("filtered_cloud", filtered_cloud);

  // Store clock
  clock_ = this->get_clock();

  std::cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> sub_callback =
    std::bind(&MultipleObjectTrackingLidar::cloud_cb, this, std::placeholders::_1);

  auto qos = rclcpp::SensorDataQoS();  // KeepLast(5) + BestEffort + Volatile
  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        filtered_cloud,
        qos,
        sub_callback);

  // Create a ROS publisher for the output point cloud
  pub_cluster0 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_0", 1);
  pub_cluster1 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_1", 1);
  pub_cluster2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_2", 1);
  pub_cluster3 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_3", 1);
  pub_cluster4 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_4", 1);
  pub_cluster5 = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_5", 1);

  objID_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("obj_id", 1);

  // MarkerArray 퍼블리셔 생성
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("viz", 10);
}

// calculate euclidean distance of two points
double MultipleObjectTrackingLidar::euclidean_distance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}

/*
objID: vector containing the IDs of the clusters that should be associated with
each KF_Tracker objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/

std::pair<int, int> MultipleObjectTrackingLidar::findIndexOfMin(std::vector<std::vector<float>> distMat) {
  std::pair<int, int> minIndex{0,0};
  float minEl = std::numeric_limits<float>::max();
  for (std::size_t i = 0; i < distMat.size(); ++i) {
    for (std::size_t j = 0; j < distMat.at(0).size(); ++j) {
      if (distMat[i][j] < minEl) {
        minEl = distMat[i][j];
        minIndex = std::make_pair(static_cast<int>(i), static_cast<int>(j));
      }
    }
  }
  return minIndex;
}

void MultipleObjectTrackingLidar::kft(const std_msgs::msg::Float32MultiArray ccs) {

  // First predict, to update the internal statePre variable
  std::vector<cv::Mat> pred{KF0.predict(), KF1.predict(), KF2.predict(),
                            KF3.predict(), KF4.predict(), KF5.predict()};

  // Get measurements
  std::vector<geometry_msgs::msg::Point> clusterCenters; // clusterCenters

  int i = 0;
  for (std::vector<float>::const_iterator it = ccs.data.begin();
       it != ccs.data.end(); it += 3) {
    geometry_msgs::msg::Point pt;
    pt.x = *it;
    pt.y = *(it + 1);
    pt.z = *(it + 2);
    clusterCenters.push_back(pt);
  }

  std::vector<geometry_msgs::msg::Point> KFpredictions;
  i = 0;
  for (auto it = pred.begin(); it != pred.end(); it++) {
    geometry_msgs::msg::Point pt;
    pt.x = (*it).at<float>(0);
    pt.y = (*it).at<float>(1);
    pt.z = 0.0f;  // 2D KF
    KFpredictions.push_back(pt);
  }

  // Find the cluster that is more probable to be belonging to a given KF.
  objID.clear();
  objID.resize(6);

  std::vector<geometry_msgs::msg::Point> copyOfClusterCenters(clusterCenters);
  std::vector<std::vector<float>> distMat;

  const float gate = 1.5f;

  for (int filterN = 0; filterN < 6; filterN++) {
    std::vector<float> distVec;
    for (int n = 0; n < 6; n++) {
      const auto &meas = copyOfClusterCenters[n];

      const bool is_dummy = (meas.x == 0.0f && meas.y == 0.0f && meas.z == 0.0f);

      const float dx = KFpredictions[filterN].x - meas.x;
      const float dy = KFpredictions[filterN].y - meas.y;
      float d = std::sqrt(dx*dx + dy*dy);

      if (is_dummy || d > gate) d = 1e6f;

      distVec.push_back(d);
    }
    distMat.push_back(distVec);
    std::cout << "filterN=" << filterN << "\n";
  }

  std::cout << "distMat.size()" << distMat.size() << "\n";
  std::cout << "distMat[0].size()" << distMat.at(0).size() << "\n";
  for (const auto &row : distMat) {
    for (const auto &s : row) std::cout << s << ' ';
    std::cout << std::endl;
  }

  for (int clusterCount = 0; clusterCount < 6; clusterCount++) {
    std::pair<int, int> minIndex(findIndexOfMin(distMat));
    std::cout << "Received minIndex=" << minIndex.first << "," << minIndex.second << "\n";

    objID[minIndex.first] = minIndex.second;

    distMat[minIndex.first] = std::vector<float>(6, 10000.0f);
    for (std::size_t row = 0; row < distMat.size(); ++row) {
      distMat[row][static_cast<std::size_t>(minIndex.second)] = 10000.0f;
    }
    std::cout << "clusterCount=" << clusterCount << "\n";
  }

  visualization_msgs::msg::MarkerArray cluster_markers;

  for (int i = 0; i < 6; i++) {
    visualization_msgs::msg::Marker m;
    m.id = i;
    m.type = visualization_msgs::msg::Marker::CUBE;

    // ====== 수정: frame_id 파라미터와 현재 시간 사용 ======
    m.header.frame_id = frame_id;            // 기존: last_input_frame_id_
    m.header.stamp    = clock_->now();       // 기존: last_input_stamp_
    // =====================================================

    m.ns = "cluster_ns";
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = i % 2 ? 1 : 0;
    m.color.g = i % 3 ? 1 : 0;
    m.color.b = i % 4 ? 1 : 0;

    geometry_msgs::msg::Point clusterC(KFpredictions[static_cast<std::size_t>(i)]);
    m.pose.position.x = clusterC.x;
    m.pose.position.y = clusterC.y;
    m.pose.position.z = clusterC.z;

    cluster_markers.markers.push_back(m);
  }

  prevClusterCenters = clusterCenters;

  marker_pub_->publish(cluster_markers);

  std_msgs::msg::Int32MultiArray obj_id;
  for (auto it = objID.begin(); it != objID.end(); it++)
    obj_id.data.push_back(*it);
  objID_pub->publish(obj_id);

  std::vector<std::vector<float>> cc;
  for (int i = 0; i < 6; i++) {
    std::vector<float> pt;
    pt.push_back(clusterCenters[objID[i]].x);
    pt.push_back(clusterCenters[objID[i]].y);
    pt.push_back(clusterCenters[objID[i]].z);
    cc.push_back(pt);
  }

  float meas0[2] = {cc[0].at(0), cc[0].at(1)};
  float meas1[2] = {cc[1].at(0), cc[1].at(1)};
  float meas2[2] = {cc[2].at(0), cc[2].at(1)};
  float meas3[2] = {cc[3].at(0), cc[3].at(1)};
  float meas4[2] = {cc[4].at(0), cc[4].at(1)};
  float meas5[2] = {cc[5].at(0), cc[5].at(1)};

  cv::Mat meas0Mat = cv::Mat(2, 1, CV_32F, meas0);
  cv::Mat meas1Mat = cv::Mat(2, 1, CV_32F, meas1);
  cv::Mat meas2Mat = cv::Mat(2, 1, CV_32F, meas2);
  cv::Mat meas3Mat = cv::Mat(2, 1, CV_32F, meas3);
  cv::Mat meas4Mat = cv::Mat(2, 1, CV_32F, meas4);
  cv::Mat meas5Mat = cv::Mat(2, 1, CV_32F, meas5);

  if (!(meas0Mat.at<float>(0, 0) == 0.0f || meas0Mat.at<float>(1, 0) == 0.0f))
    cv::Mat estimated0 = KF0.correct(meas0Mat);
  if (!(meas1[0] == 0.0f || meas1[1] == 0.0f))
    cv::Mat estimated1 = KF1.correct(meas1Mat);
  if (!(meas2[0] == 0.0f || meas2[1] == 0.0f))
    cv::Mat estimated2 = KF2.correct(meas2Mat);
  if (!(meas3[0] == 0.0f || meas3[1] == 0.0f))
    cv::Mat estimated3 = KF3.correct(meas3Mat);
  if (!(meas4[0] == 0.0f || meas4[1] == 0.0f))
    cv::Mat estimated4 = KF4.correct(meas4Mat);
  if (!(meas5[0] == 0.0f || meas5[1] == 0.0f))
    cv::Mat estimated5 = KF5.correct(meas5Mat);
}

void MultipleObjectTrackingLidar::publish_cloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  auto clustermsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cluster, *clustermsg);

  // ====== 수정: frame_id 파라미터와 현재 시간 사용 ======
  clustermsg->header.frame_id = frame_id;   // 기존: last_input_frame_id_
  clustermsg->header.stamp    = clock_->now(); // 기존: last_input_stamp_
  // =====================================================

  pub->publish(*clustermsg);
}

void MultipleObjectTrackingLidar::cloud_cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input)
{
  // 입력 메시지의 프레임/스탬프는 여전히 저장(참고용)
  last_input_frame_id_ = input->header.frame_id;
  last_input_stamp_    = input->header.stamp;

  if (firstFrame) {
    // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.

    float dvx = 0.01f;
    float dvy = 0.01f;
    float dx = 1.0f;
    float dy = 1.0f;

    KF0.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
    KF1.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
    KF2.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
    KF3.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
    KF4.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
    KF5.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);

    cv::setIdentity(KF0.measurementMatrix);
    cv::setIdentity(KF1.measurementMatrix);
    cv::setIdentity(KF2.measurementMatrix);
    cv::setIdentity(KF3.measurementMatrix);
    cv::setIdentity(KF4.measurementMatrix);
    cv::setIdentity(KF5.measurementMatrix);

    float sigmaP = 0.01f;
    float sigmaQ = 0.1f;
    cv::setIdentity(KF0.processNoiseCov, cv::Scalar::all(sigmaP));
    cv::setIdentity(KF1.processNoiseCov, cv::Scalar::all(sigmaP));
    cv::setIdentity(KF2.processNoiseCov, cv::Scalar::all(sigmaP));
    cv::setIdentity(KF3.processNoiseCov, cv::Scalar::all(sigmaP));
    cv::setIdentity(KF4.processNoiseCov, cv::Scalar::all(sigmaP));
    cv::setIdentity(KF5.processNoiseCov, cv::Scalar::all(sigmaP));

    cv::setIdentity(KF0.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF1.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF2.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF3.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF4.measurementNoiseCov, cv::Scalar(sigmaQ));
    cv::setIdentity(KF5.measurementNoiseCov, cv::Scalar(sigmaQ));

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      float x = 0.0f, y = 0.0f, z = 0.0f;
      int numPts = 0;

      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        cloud_cluster->points.push_back(input_cloud->points[*pit]);
        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        z += input_cloud->points[*pit].z;
        numPts++;
      }

      pcl::PointXYZ centroid;
      centroid.x = (numPts ? x/numPts : 0.0f);
      centroid.y = (numPts ? y/numPts : 0.0f);
      centroid.z = (numPts ? z/numPts : 0.0f);

      cluster_vec.push_back(cloud_cluster);
      clusterCentroids.push_back(centroid);
    }

    while (cluster_vec.size() < 6) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < 6) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0f; centroid.y = 0.0f; centroid.z = 0.0f;
      clusterCentroids.push_back(centroid);
    }

    KF0.statePre.at<float>(0) = clusterCentroids.at(0).x;
    KF0.statePre.at<float>(1) = clusterCentroids.at(0).y;
    KF0.statePre.at<float>(2) = 0;
    KF0.statePre.at<float>(3) = 0;

    KF1.statePre.at<float>(0) = clusterCentroids.at(1).x;
    KF1.statePre.at<float>(1) = clusterCentroids.at(1).y;
    KF1.statePre.at<float>(2) = 0;
    KF1.statePre.at<float>(3) = 0;

    KF2.statePre.at<float>(0) = clusterCentroids.at(2).x;
    KF2.statePre.at<float>(1) = clusterCentroids.at(2).y;
    KF2.statePre.at<float>(2) = 0;
    KF2.statePre.at<float>(3) = 0;

    KF3.statePre.at<float>(0) = clusterCentroids.at(3).x;
    KF3.statePre.at<float>(1) = clusterCentroids.at(3).y;
    KF3.statePre.at<float>(2) = 0;
    KF3.statePre.at<float>(3) = 0;

    KF4.statePre.at<float>(0) = clusterCentroids.at(4).x;
    KF4.statePre.at<float>(1) = clusterCentroids.at(4).y;
    KF4.statePre.at<float>(2) = 0;
    KF4.statePre.at<float>(3) = 0;

    KF5.statePre.at<float>(0) = clusterCentroids.at(5).x;
    KF5.statePre.at<float>(1) = clusterCentroids.at(5).y;
    KF5.statePre.at<float>(2) = 0;
    KF5.statePre.at<float>(3) = 0;

    firstFrame = false;

    for (int i = 0; i < 6; i++) {
      geometry_msgs::msg::Point pt;
      pt.x = clusterCentroids.at(i).x;
      pt.y = clusterCentroids.at(i).y;
      prevClusterCenters.push_back(pt);
    }
  } else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *input_cloud);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.08);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(600);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;
    std::vector<pcl::PointXYZ> clusterCentroids;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
      float x = 0.0f, y = 0.0f, z = 0.0f;
      int numPts = 0;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {
        cloud_cluster->points.push_back(input_cloud->points[*pit]);
        x += input_cloud->points[*pit].x;
        y += input_cloud->points[*pit].y;
        z += input_cloud->points[*pit].z;
        numPts++;
      }

      pcl::PointXYZ centroid;
      centroid.x = (numPts ? x/numPts : 0.0f);
      centroid.y = (numPts ? y/numPts : 0.0f);
      centroid.z = (numPts ? z/numPts : 0.0f);

      cluster_vec.push_back(cloud_cluster);
      clusterCentroids.push_back(centroid);
    }

    while (cluster_vec.size() < 6) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
      cluster_vec.push_back(empty_cluster);
    }

    while (clusterCentroids.size() < 6) {
      pcl::PointXYZ centroid;
      centroid.x = 0.0f; centroid.y = 0.0f; centroid.z = 0.0f;
      clusterCentroids.push_back(centroid);
    }

    std_msgs::msg::Float32MultiArray cc;
    for (int i = 0; i < 6; i++) {
      cc.data.push_back(clusterCentroids.at(i).x);
      cc.data.push_back(clusterCentroids.at(i).y);
      cc.data.push_back(clusterCentroids.at(i).z);
    }

    kft(cc);

    int i = 0;
    bool publishedCluster[6];
    for (auto it = objID.begin(); it != objID.end(); it++) {
      switch (i) {
      case 0: {
        publish_cloud(pub_cluster0, cluster_vec[*it]);
        publishedCluster[i] = true;
        i++;
        break;
      }
      case 1: {
        publish_cloud(pub_cluster1, cluster_vec[*it]);
        publishedCluster[i] = true;
        i++;
        break;
      }
      case 2: {
        publish_cloud(pub_cluster2, cluster_vec[*it]);
        publishedCluster[i] = true;
        i++;
        break;
      }
      case 3: {
        publish_cloud(pub_cluster3, cluster_vec[*it]);
        publishedCluster[i] = true;
        i++;
        break;
      }
      case 4: {
        publish_cloud(pub_cluster4, cluster_vec[*it]);
        publishedCluster[i] = true;
        i++;
        break;
      }
      case 5: {
        publish_cloud(pub_cluster5, cluster_vec[*it]);
        publishedCluster[i] = true;
        i++;
        break;
      }
      default:
        break;
      }
    }
  }
}

} // namespace multiple_object_tracking_lidar
