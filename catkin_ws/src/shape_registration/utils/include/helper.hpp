#ifndef HELPER_HPP
#define HELPER_HPP

// Eigen related
#include <Eigen/Geometry>
// YAML related
#include <yaml-cpp/yaml.h>

// std
#include <fstream>
#include <iostream>

// ros
#include <geometry_msgs/TransformStamped.h>


namespace Utils {

/**
 * @brief from_quaternion_to_rot a function which creates rotation matrix from quaternion
 * @param q
 * @return
 */
template<typename T>
Eigen::Matrix<T, 3, 3> from_quaternion_to_rot(Eigen::Quaternion<T> &q) {
  return q.normalized().toRotationMatrix();
}

template<typename T>
Eigen::Matrix<T, 4, 4> from_rotation_translation_to_transformation(const Eigen::Matrix<T, 3, 3> &rotation, const Eigen::Matrix<T, 3, 1> &translation){
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

  transformation.block<3, 3>(0, 0) = rotation;
  transformation.block<3, 1>(0, 3) = translation;
  return transformation;
}

}
#endif // HELPER_HPP
