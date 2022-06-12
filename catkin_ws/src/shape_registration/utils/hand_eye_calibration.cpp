#include <iostream>
#include "icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>

PointCloudT::Ptr read_file(std::string file_path) {
    PointCloudT::Ptr result (new PointCloudT);
    std::string s;
    std::ifstream points_file (file_path);
    unsigned int num_points = 0 ;
    if (points_file.is_open())
    {
      while ( getline (points_file,s) )
      {
        PointT point;
        std::vector<float> coords;
        // parse the string
        std::string delimiter = " ";
        size_t pos = 0;
        std::string token;
        while ((pos = s.find(delimiter)) != std::string::npos) {
            token = s.substr(0, pos);
            coords.push_back(std::stof(token));
            s.erase(0, pos + delimiter.length());
        }
        coords.push_back(std::stof(s));
        point.x = coords[0]*1000.0f;
        point.y = coords[1] * 1000.0f;
        point.z = coords[2] * 1000.0f;
        result->points.push_back(point);
        num_points += 1;
      }
      result->height = 1;
      result->width = num_points;
      points_file.close();
    }
    return result;
}


int main() {

  // Generate icp algorithm object
  auto icp_ = std::make_shared<ICPAlgorithm>(1000);

  PointCloudT::Ptr result (new PointCloudT);
  auto coords_in_cam = read_file("/home/y/RosCode/catkin_ws/src/dataset/robot.txt");
  auto coords_in_robot = read_file("/home/y/RosCode/catkin_ws/src/dataset/corners_pos.txt");


  icp_->find_initial_transform_for_small_sets(coords_in_cam, coords_in_robot);

  pcl::transformPointCloud(*coords_in_cam, *result, icp_->transformation);
  for(const auto &point : result->points) {
    std::cout << point << std::endl;
  }


  pcl::visualization::PCLVisualizer::Ptr viewer  = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
  viewer->setBackgroundColor (255, 250, 250);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> ct_arm_color (coords_in_cam, 0, 0, 255);
  viewer->addPointCloud<PointT> (coords_in_cam, ct_arm_color, "coords_in_cam");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "coords_in_cam");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> result_color (result, 0, 255, 0);
  viewer->addPointCloud<PointT> (result, result_color, "result");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "result");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> target_cloud_color (coords_in_robot, 255, 0, 0);
  viewer->addPointCloud<PointT> (coords_in_robot, target_cloud_color, "coords_in_robot");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "coords_in_robot");

  while (!viewer->wasStopped ())
  {
      viewer->spinOnce (100);
  }

  auto distance = 0.0f;
  for (unsigned int i = 0 ; i < coords_in_cam->points.size(); i ++) {
    auto gt = coords_in_robot->points[i];
    auto calc = result->points[i];
    auto ind_error =  std::sqrt(std::pow(gt.x - calc.x, 2) + std::pow(gt.y - calc.y, 2) + std::pow(gt.z - calc.z, 2));
    std::cout << ind_error <<std::endl;
    distance += ind_error;
  }

  std::cout << "Error: " << distance * 1000.0f/ float(coords_in_cam->points.size()) << " mm" << std::endl;

  auto transformation = icp_->transformation;
  std::cout << transformation << std::endl;
  std::ofstream file("/home/y/RosCode/catkin_ws/src/dataset/R2K.txt");
  for (size_t i = 0; i < 4; i++)
  {
    for (size_t j = 0; j < 4; j++)
    {
      file<<transformation(i,j)<<" ";
    }
    file<<std::endl;    
  }
  
//  Eigen::Quaternionf q(transformation.block<3,3>(0,0));
//  std::cout << "q.x()" << q.x() << std::endl;
//  std::cout << "q.y()" << q.y() << std::endl;
//  std::cout << "q.z()" << q.z() << std::endl;
//  std::cout << "q.w()" << q.w() << std::endl;
//  std::cout << "x()" << transformation(0, 3) << std::endl;
//  std::cout << "y()" << transformation(1, 3) << std::endl;
//  std::cout << "z()" << transformation(2, 3) << std::endl;

  return 0;
}
