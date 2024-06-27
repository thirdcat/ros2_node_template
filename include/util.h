#ifndef _UTIL_H_
#define _UTIL_H_

#include <Eigen/Eigen>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>

inline Eigen::Matrix4f t_from_values(const float& p_x, const float& p_y,
                                     const float& p_z, const float& q_w,
                                     const float& q_x, const float& q_y,
                                     const float& q_z) {
  Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
  t.block<3, 3>(0, 0) =
      Eigen::Quaternionf(q_w, q_x, q_y, q_z).toRotationMatrix();
  t.block<3, 1>(0, 3) = Eigen::Vector3f(p_x, p_y, p_z);
  return t;
}

inline Eigen::Matrix4f t_from_pose(const geometry_msgs::msg::Pose& pose) {
  return t_from_values(pose.position.x, pose.position.y, pose.position.z,
                       pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
}

inline Eigen::Matrix4f t_from_pose_with_covariance_stamped(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
  return t_from_pose(pose.pose.pose);
}

inline Eigen::Matrix4f t_from_tf(
    const geometry_msgs::msg::TransformStamped& tf) {
  return t_from_values(tf.transform.translation.x, tf.transform.translation.y,
                       tf.transform.translation.z, tf.transform.rotation.w,
                       tf.transform.rotation.x, tf.transform.rotation.y,
                       tf.transform.rotation.z);
}

inline geometry_msgs::msg::TransformStamped tf_from_values(
    const float& p_x, const float& p_y, const float& p_z, const float& q_w,
    const float& q_x, const float& q_y, const float& q_z) {
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.translation.x = p_x;
  tf.transform.translation.y = p_y;
  tf.transform.translation.z = p_z;
  tf.transform.rotation.x = q_x;
  tf.transform.rotation.y = q_y;
  tf.transform.rotation.z = q_z;
  tf.transform.rotation.w = q_w;
  return tf;
}

inline geometry_msgs::msg::TransformStamped tf_from_t(
    const Eigen::Matrix4f& t) {
  Eigen::Quaternionf q(t.block<3, 3>(0, 0));
  return tf_from_values(t(0, 3), t(1, 3), t(2, 3), q.w(), q.x(), q.y(), q.z());
}

inline std::string to_string(const Eigen::Matrix4f& t) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "p: " << t.block<3, 1>(0, 3).transpose() << " rpy: "
     << t.block<3, 3>(0, 0).eulerAngles(0, 1, 2).transpose() * 180.0 / M_PI;
  return ss.str();
}

inline std::string to_string(const Eigen::Isometry3d& t) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3)
     << "p: " << t.translation().transpose()
     << " rpy: " << t.linear().eulerAngles(0, 1, 2).transpose() * 180.0 / M_PI;
  return ss.str();
}

// Serialization methods for fixed or dynamic-size Eigen::Matrix type
namespace boost {
namespace serialization {
template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options,
          int _MaxRows, int _MaxCols>
inline void serialize(
    Archive& ar,
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix,
    const unsigned int /* aVersion */) {
  Eigen::Index rows = matrix.rows();
  Eigen::Index cols = matrix.cols();
  ar&(rows);
  ar&(cols);
  if (rows != matrix.rows() || cols != matrix.cols())
    matrix.resize(rows, cols);
  if (matrix.size() != 0)
    ar& boost::serialization::make_array(matrix.data(), rows * cols);
}
}  // namespace serialization
}  // namespace boost

inline void save_covs_to_binary(
    const std::vector<Eigen::Matrix4d,
                      Eigen::aligned_allocator<Eigen::Matrix4d>>& map_covs,
    const std::string& filename, bool use_half = false) {
  std::ofstream ofs(filename);
  boost::archive::binary_oarchive oa(ofs);
  if (use_half) {
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        map_covs_half;
    map_covs_half.reserve(map_covs.size());
    for (const auto& cov : map_covs) {
      Eigen::Matrix4f cov_half = cov.cast<float>();
      map_covs_half.push_back(cov_half);
    }
    oa & map_covs_half;
  } else {
    oa & map_covs;
  }

  ofs.close();
  return;
}

inline void load_covs_from_binary(
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>&
        map_covs,
    const std::string& filename, bool use_half = false) {
  std::ifstream ifs(filename);
  boost::archive::binary_iarchive ia(ifs);
  if (use_half) {
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
        map_covs_half;
    ia & map_covs_half;
    map_covs.reserve(map_covs_half.size());
    for (const auto& cov : map_covs_half) {
      Eigen::Matrix4d cov_double = cov.cast<double>();
      map_covs.push_back(cov_double);
    }
  } else {
    ia & map_covs;
  }

  ifs.close();
  return;
}

inline std::string to_string_with_precision(const double a_value,
                                            const int n = 6) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(n) << a_value;
  return out.str();
}

inline bool is_prime(int n) {
  if (n < 2)
    return false;
  for (int i = 2; i < n; ++i) {
    if ((n % i) == 0)
      return false;
  }
  return true;
}

#endif
