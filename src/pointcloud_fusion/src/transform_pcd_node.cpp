#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
  // 1. 포인트 클라우드 객체 생성
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // 2. lidar2.pcd 파일 읽기
  pcl::PCDReader reader;
  if (reader.read<pcl::PointXYZ>("lidar2.pcd", *source_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file lidar2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << source_cloud->width * source_cloud->height << " data points from lidar2.pcd." << std::endl;

  // 3. 변환 행렬 정의
  // Eigen 라이브러리를 사용하여 4x4 변환 행렬을 만듭니다.
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  // 이동(Translation) 값 설정 (x=0, y=0.7, z=0)
  transform(0, 3) = -0.3;  // x
  transform(1, 3) = -0.53; // y
  transform(2, 3) = 0.0;   // z  
  
  // 회전(Rotation) 값 설정 (라디안 단위)
  float yaw_angle = -0.524; // yaw
  float pitch_angle = 0.0;   // pitch
  float roll_angle = 0.0;    // roll

  Eigen::AngleAxisf roll(roll_angle, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitch(pitch_angle, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yaw(yaw_angle, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q = yaw * pitch * roll;
  transform.block<3,3>(0,0) = q.toRotationMatrix();

  std::cout << "Applying transform:" << std::endl << transform << std::endl;

  // 4. 포인트 클라우드 변환 적용
  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);

  // 5. 변환된 클라우드를 새 파일로 저장
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>("lidar2_transformation.pcd", *transformed_cloud, false);

  std::cout << "Saved " << transformed_cloud->points.size() << " data points to lidar2_transformation.pcd." << std::endl;

  return 0;
}