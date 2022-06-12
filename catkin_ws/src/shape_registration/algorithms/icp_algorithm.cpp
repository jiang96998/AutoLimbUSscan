#include "icp_algorithm.hpp"
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <string>
#include <cmath>
#include <pcl/registration/default_convergence_criteria.h>

ICPAlgorithm::ICPAlgorithm(int max_num_iter)
{
  ROS_INFO("hey hey hey initializing");
  this->m_max_num_iter = max_num_iter;
}

void ICPAlgorithm::get_initial_transformation(PointCloudT::Ptr &source, PointCloudT::Ptr &target) {
  if(!this->m_corr_filtered.empty()) {
    this->m_corr_filtered.clear();
  }

  // Before applying icp, it is better to find an initial alignment, between the clouds.

//  double start_calc_normals =ros::Time::now().toSec();
  /**
    Calculate the normals for source
    */
   PointCloudNormal::Ptr source_normals;
   calculateNormals(source, source_normals);

  /**
    Calculate the normals for target
    */
   PointCloudNormal::Ptr target_normals;
   calculateNormals(target, target_normals);

//   double finish_calc_normals =ros::Time::now().toSec();
//   ROS_INFO("Time takes for calculating normals of the target and source clouds: %f", finish_calc_normals - start_calc_normals);

  /**
    Now we want to find the persistent keypoints from both source and target clouds in different scales.
    Then we can use these keypoints and their features while finding the initial alignment.
    */

//   double start_calc_persistent_feat =ros::Time::now().toSec();
   /**
     FIND PERSISTENT FEATURES FOR THE SOURCE CLOUD
     */

   FeatureCloud::Ptr source_features(new FeatureCloud());
   auto source_keypoints_indices = boost::shared_ptr<std::vector<int>>();
   find_multiscale_persistent_features(source, source_normals, source_features, source_keypoints_indices, m_scale_values_MRI, m_alpha_MRI);
   PointCloudT::Ptr source_keypoints(new PointCloudT);
   PointCloudT::Ptr source_non_keypoints(new PointCloudT);
   Preprocessing::extract_indices(source, source_keypoints_indices, source_keypoints, source_non_keypoints);
   /**
    FIND PERSISTENT FEATURES FOR THE TARGET CLOUD
    */

   FeatureCloud::Ptr target_features(new FeatureCloud());
   auto target_keypoints_indices = boost::shared_ptr<std::vector<int>>();
   find_multiscale_persistent_features(target, target_normals, target_features, target_keypoints_indices, m_scale_values_Kinect, m_alpha_kinect);
   PointCloudT::Ptr target_keypoints(new PointCloudT);
   PointCloudT::Ptr target_non_keypoints(new PointCloudT);
   Preprocessing::extract_indices(target, target_keypoints_indices, target_keypoints, target_non_keypoints);
//   double finish_calc_persistent_feat =ros::Time::now().toSec();

//   ROS_INFO("Time takes for calculating persistent features: %f", finish_calc_persistent_feat - start_calc_persistent_feat);
   ROS_INFO("The number of persistent features found in source: %zu", source_keypoints->size());
   ROS_INFO("The number of persistent features found in target: %zu", target_keypoints->size());

   /**
    Now that we have the keypoints along with their features both from source and target, we need to estimate the correspondance and
    drop the ones which are not likely.
    */

//   double start_calc_correspondences =ros::Time::now().toSec();

   pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
   pcl::registration::CorrespondenceEstimation<Feature, Feature> cest;

   cest.setInputSource(source_features);
   cest.setInputTarget(target_features);
   cest.determineCorrespondences(*correspondences);

   pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> rejector;
   pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
   //this->m_corr_filtered = std::shared_ptr<pcl::Correspondences>();
   rejector.setInputSource(source_keypoints);
   rejector.setInputTarget(target_keypoints);
   rejector.setInlierThreshold(0.25);
   rejector.setMaximumIterations(1000000);
   rejector.setRefineModel(false);
   rejector.setInputCorrespondences(correspondences);
   rejector.getCorrespondences(*corr_filtered);

   // The below line somehow didnt work!
   //estimate_correspondances(source_features, target_features, source_keypoints, target_keypoints, corr_filtered);

//   double finish_calc_correspondences =ros::Time::now().toSec();

//   ROS_INFO("Time takes for calculating the feature correspondences: %f", finish_calc_correspondences - start_calc_correspondences);
   /**
    Find the initial alignment between source and the target
    */

   pcl::registration::TransformationEstimationSVD<PointT,PointT> transformation_est_SVD;
   transformation_est_SVD.estimateRigidTransformation(*source_keypoints, *target_keypoints, *corr_filtered, this->transformation);

   for(auto const &corr :*corr_filtered) {
     this->m_corr_filtered.push_back(corr);
   }

   this->m_source_keypoints = source_keypoints;
   this->m_target_keypoints = target_keypoints;
   this->m_source_non_keypoints = source_non_keypoints;
   this->m_target_non_keypoints = target_non_keypoints;
}

void ICPAlgorithm::find_initial_transform_for_small_sets(PointCloudT::Ptr &source_cloud, PointCloudT::Ptr &target_cloud) {
  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
  correspondences->resize(source_cloud->points.size());

  // Here we manually defined the correspondences!
  for(unsigned int i = 0; i < source_cloud->points.size(); i++) {
    pcl::Correspondence corr;
    corr.index_query = i;
    corr.index_match = i;
    corr.distance = 0;
    (*correspondences)[i] = corr;
  }

  pcl::registration::TransformationEstimationSVD<PointT,PointT> transformation_est_SVD;
  transformation_est_SVD.estimateRigidTransformation(*source_cloud, *target_cloud, *correspondences, this->transformation);
}

PointCloudT ICPAlgorithm::compute(PointCloudT::Ptr &source, PointCloudT::Ptr &target) {

  // Set the input source and input target point clouds to the icp algorithm.

  this->icp.setInputSource(source);
  this->icp.setInputTarget(target);

 PointCloudT final_cloud;

  // Set the maximum number of iterations
  // It is 10 by default
  //this->icp.setMaxCorrespondenceDistance (0.1);
//  this->icp.setMaximumIterations(this->m_max_num_iter);
 this->icp.setMaximumIterations(this->m_max_num_iter);
  //this->icp.setEuclideanFitnessEpsilon(1e-8);
  // Set convergence criteria
  //this->icp.setTransformationEpsilon (1e-8);

  // Create a new point cloud which will represent the result point cloud after
  // iteratively applying transformations to the input source cloud, to make it
  // look like the target point cloud.
//  PointCloudT final_cloud;

  this->icp.align(final_cloud);
//  this->icp.setMaximumIterations(1500);

  // Print whether the icp algorithm has converged to the same result with the
  // target, and the resulting rigid body transformation details.
  ROS_INFO("has converged : %d", this->icp.hasConverged());
  ROS_INFO("score : %f", this->icp.getFitnessScore());


  if(this->icp.hasConverged()) {
    return final_cloud;
  }
  final_cloud.clear();
  return final_cloud;

}

PointCloudT ICPAlgorithm::compute(PointCloudT::Ptr &source, PointCloudT::Ptr &target, double_t score) {
  auto final_cloud = compute(source, target);
  if(!final_cloud.empty() && this->icp.getFitnessScore() < score) {
    return final_cloud;
  }
  final_cloud.clear();
  return final_cloud;
}

void ICPAlgorithm::find_multiscale_persistent_features(PointCloudT::Ptr &input_cloud,
                                                       PointCloudNormal::Ptr& input_cloud_normals,
                                                       FeatureCloud::Ptr& features,
                                                       boost::shared_ptr<std::vector<int>>& indices,
                                                       std::vector<float> & scale_values, const float alpha) {
    pcl::MultiscaleFeaturePersistence<PointT, Feature> feature_persistence;
    feature_persistence.setScalesVector(scale_values);
    feature_persistence.setAlpha(alpha);
    pcl::FPFHEstimation<PointT, pcl::Normal, Feature>::Ptr fpfh_estimation(new pcl::FPFHEstimation<PointT, pcl::Normal, Feature>());
    fpfh_estimation->setInputCloud(input_cloud);
    fpfh_estimation->setInputNormals(input_cloud_normals);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    fpfh_estimation->setSearchMethod(tree);
    feature_persistence.setFeatureEstimator(fpfh_estimation);
    feature_persistence.setDistanceMetric(pcl::KL);

    feature_persistence.determinePersistentFeatures(*features, indices);

}

void ICPAlgorithm::calculateNormals(PointCloudT::Ptr& cloud_subsampled,
                                  PointCloudNormal::Ptr& cloud_subsampled_normals)
{
  cloud_subsampled_normals = PointCloudNormal::Ptr(new PointCloudNormal());
  pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud(cloud_subsampled);
  pcl::search::KdTree<PointT>::Ptr search_tree(new pcl::search::KdTree<PointT>);
  normal_estimation_filter.setSearchMethod(search_tree);
  normal_estimation_filter.setRadiusSearch(0.05);
  normal_estimation_filter.compute(*cloud_subsampled_normals);
}
