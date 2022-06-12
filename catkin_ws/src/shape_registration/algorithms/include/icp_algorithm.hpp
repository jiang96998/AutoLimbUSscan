#ifndef ICP_ALGORITHM_H
#define ICP_ALGORITHM_H

#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include "preprocessing.hpp"

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <boost/shared_ptr.hpp>

#include <vector>
//using Matrix = Eigen::Matrix<float, 4, 4>;
using PointCloudNormal = pcl::PointCloud<pcl::Normal>;
using Feature = pcl::FPFHSignature33;
using FeatureCloud = pcl::PointCloud<Feature>;


class ICPAlgorithm
{
public:

  ICPAlgorithm(int max_num_iter);
  PointCloudT compute(PointCloudT::Ptr &source, PointCloudT::Ptr &target);
  PointCloudT compute(PointCloudT::Ptr &source, PointCloudT::Ptr &target, double_t score);
  //Matrix get_initial_transformation(PointCloudT::Ptr &source, PointCloudT::Ptr &target);
  void get_initial_transformation(PointCloudT::Ptr &source, PointCloudT::Ptr &target);
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  inline PointCloudT::Ptr get_source_keypoints() { return this->m_source_keypoints; }
  inline PointCloudT::Ptr get_target_keypoints() { return this->m_target_keypoints; }
  inline PointCloudT::Ptr get_source_non_keypoints() { return this->m_source_non_keypoints; }
  inline PointCloudT::Ptr get_target_non_keypoints() { return this->m_target_non_keypoints; }
  void find_initial_transform_for_small_sets(PointCloudT::Ptr &source_cloud, PointCloudT::Ptr &target_cloud);

public:
  pcl::registration::TransformationEstimationSVD<PointT,PointT>::Matrix4 transformation;
  inline pcl::Correspondences get_final_correspondences() { return this->m_corr_filtered; }


private:

  /**
   * @brief find_multiscale_persistent_features
   * @param input_cloud
   * @param input_cloud_normals
   * @param features
   * @param keypoints
   */
  void find_multiscale_persistent_features(PointCloudT::Ptr &input_cloud,
                                           PointCloudNormal::Ptr& input_cloud_normals,
                                           FeatureCloud::Ptr& features,
                                           boost::shared_ptr<std::vector<int>>& indices,
                                           std::vector<float> & scale_values,
                                           const float alpha);

  /**
   * @brief calculateNormals
   * @param cloud_subsampled
   * @param cloud_subsampled_normals
   */
  void calculateNormals(PointCloudT::Ptr& cloud_subsampled,
                                    PointCloudNormal::Ptr& cloud_subsampled_normals);

private:
  int m_max_num_iter;
  std::vector<float> m_scale_values_MRI{1.0/100.0, 1.5/100.0, 2.0/100.0, 2.5/100.0, 3.0/100.0};
  std::vector<float> m_scale_values_Kinect{1.0/100.0, 1.5/100.0, 2.0/100.0, 2.5/100.0, 3.0/100.0};
  float m_alpha_MRI = 0.9f;
  float m_alpha_kinect = 0.7f;
  PointCloudT::Ptr m_source_keypoints;
  PointCloudT::Ptr m_target_keypoints;
  PointCloudT::Ptr m_source_non_keypoints;
  PointCloudT::Ptr m_target_non_keypoints;
  pcl::Correspondences m_corr_filtered;
};

#endif // ICP_ALGORITHM_H
