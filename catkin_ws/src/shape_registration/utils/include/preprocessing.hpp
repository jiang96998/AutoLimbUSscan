#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

namespace Preprocessing {

/**
 * @brief pass_through_filter
 * @param input_cloud
 * @param x_min_val
 * @param x_max_val
 * @param y_min_val
 * @param y_max_val
 * @param z_min_val
 * @param z_max_val
 * @return filtered cloud
 */
PointCloudT::Ptr pass_through_filter(PointCloudT::Ptr &input_cloud, float x_min_val, float x_max_val, float y_min_val, float y_max_val, float z_min_val, float z_max_val);

/**
 * @brief voxel_grid_downsampling
 * @param input_cloud
 * @param voxel_size
 * @return
 */
PointCloudT::Ptr voxel_grid_downsampling(PointCloudT::Ptr &input_cloud, float voxel_size);

/**
 * @brief statistical_filtering
 * @param input_cloud
 * @return
 */
PointCloudT::Ptr statistical_filtering(PointCloudT::Ptr &input_cloud, double std_dev);

/**
 * @brief extract_plane
 * @param input_cloud
 * @param threshold
 * @return
 */
PointCloudT::Ptr extract_plane(PointCloudT::Ptr &input_cloud, double threshold);

void extract_indices(const PointCloudT::Ptr& source,
                          boost::shared_ptr<std::vector<int>>& indices_to_extract,
                          PointCloudT::Ptr& output_inliners, PointCloudT::Ptr& output_outliners);

void visualize_point_cloud(const PointCloudT &source);


/**
 * @brief extract_indices
 * @param source
 * @param indices_to_extract
 * @param output
 */
void extract_indices(const PointCloudT::Ptr& source, boost::shared_ptr<std::vector<int>>& indices_to_extract, PointCloudT::Ptr& output);


void euclidean_clustering_pcl(const PointCloudT::Ptr& cloud);

}


#endif // PREPROCESSING_HPP
