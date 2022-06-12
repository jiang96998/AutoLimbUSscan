#include "preprocessing.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

// The methods used in preprocessing of the data from Azure cloud is kept here

//
namespace Preprocessing {

PointCloudT::Ptr pass_through_filter(PointCloudT::Ptr &input_cloud, float x_min_val, float x_max_val, float y_min_val, float y_max_val, float z_min_val, float z_max_val) {

  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  // create a pass through filter object
  // the parameter to the object constructer is false, if you make it true
  // it will extract out the parts that are filtered, after the filtering
  // is complete.
  pcl::PassThrough<PointT> pass(false);

  // Apply filtering in the z dimension
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min_val, z_max_val);
  pass.filter(*cloud_filtered);

  // Apply filtering in the y dimension
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_min_val, y_max_val);
  pass.filter(*cloud_filtered);

  //Apply filtering in the x dimension
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_min_val, x_max_val);
  pass.filter(*cloud_filtered);

  return cloud_filtered;
}

void euclidean_clustering_pcl(const PointCloudT::Ptr& cloud) {
  // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.04); // 2cm
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (1700);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::cout << cluster_indices.size() << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud)[idx]); //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      pcl::io::savePCDFileASCII ("/home/nehil/catkin_ws_registration/src/" + ss.str(), *cloud_cluster);
      j++;
    }
}

PointCloudT::Ptr voxel_grid_downsampling(PointCloudT::Ptr &input_cloud, float voxel_size) {
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  // Create the filtering object
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(input_cloud);
  sor.setLeafSize(voxel_size, voxel_size, voxel_size);
  sor.filter(*cloud_filtered);

    return cloud_filtered;
}


PointCloudT::Ptr statistical_filtering(PointCloudT::Ptr &input_cloud, double std_dev) {
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (input_cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (std_dev);
  sor.filter (*cloud_filtered);
  std::cout << "Applied the filter" << std::endl;
  std::cout << cloud_filtered->points.size() << std::endl;

  return cloud_filtered;
}

void extract_indices(const PointCloudT::Ptr& source,
                          boost::shared_ptr<std::vector<int>>& indices_to_extract,
                          PointCloudT::Ptr& output){

  // Extract inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(source);
  extract.setIndices(indices_to_extract);
  extract.setNegative(false);     // Extract the inliers
  extract.filter(*output); // cloud_inliers contains the plane

}

void extract_indices(const PointCloudT::Ptr& source,
                          boost::shared_ptr<std::vector<int>>& indices_to_extract,
                          PointCloudT::Ptr& output_inliners, PointCloudT::Ptr& output_outliners){
  // Extract inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(source);
  extract.setIndices(indices_to_extract);
  extract.setNegative(false);     // Extract the inliers
  extract.filter(*output_inliners); // cloud_inliers contains the plane

  extract.setNegative(true);     // Extract the outliners
  extract.filter(*output_outliners);

}

PointCloudT::Ptr extract_plane(PointCloudT::Ptr &input_cloud, double threshold) {

  PointCloudT::Ptr cloud_inliers(new PointCloudT);
  PointCloudT::Ptr cloud_outliers(new PointCloudT);
  PointCloudT::Ptr cloud_outliers_one_side_plane(new PointCloudT);
  pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

  // The plane equation will have 4 coefficients
  plane->values.resize(4);

  // Create a segmentation object
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setDistanceThreshold(threshold);
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers_plane, *plane);

  if (inliers_plane->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    return nullptr;
  }

  // Extract inliers
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers_plane);
  extract.setNegative(false);     // Extract the inliers
  extract.filter(*cloud_inliers); // cloud_inliers contains the plane


  // Extract outliers
  extract.setNegative(true); // Extract the outliers
  extract.filter(*cloud_outliers);

  auto a = plane->values[0];
  auto b = plane->values[1];
  auto c = plane->values[2];
  auto d = plane->values[3];
  //unsigned int num_points = 0;
  Eigen::Vector3f normal_vec(a, b, c);
  Eigen::Vector3f cam_y(0, -1, 0);
  auto dot_product = normal_vec.transpose() * cam_y;
  bool larger = false;
  if(dot_product > 0) {
    // normal vector is showing up
    larger = true;
  }
  for(const auto &point : cloud_outliers->points) {
    float plane_side = a * point._PointXYZ::x + b * point._PointXYZ::y + c * point._PointXYZ::z + d;
    if(larger) {
      if(plane_side > 0) {
        cloud_outliers_one_side_plane->points.push_back(point);
        //num_points++;
      }
    }
    else{
      if(plane_side <= 0) {
        cloud_outliers_one_side_plane->points.push_back(point);
        //num_points++;
      }
    }
  }

  //cloud_outliers_one_side_plane->width = num_points;
  //cloud_outliers_one_side_plane->height = 1;

  //return  cloud_outliers_one_side_plane;
  return cloud_outliers_one_side_plane;
}

void visualize_point_cloud(const PointCloudT &source) {
    PointCloudT::Ptr source_ptr(new PointCloudT(source));
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // = pcl::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");
    viewer->setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> rgb_final(source_ptr, 0, 0, 255);
      //pcl::make_shared<PointCloudT>(source), 0, 0, 255);
    viewer->addPointCloud<PointT> (source_ptr, rgb_final, "random_cloud");
      //pcl::make_shared<PointCloudT>(source), rgb_final, "random_cloud");
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }
}

} // Preprocessing namespace.
