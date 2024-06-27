#ifndef ROS2_NODE_TEMPLATE_H_
#define ROS2_NODE_TEMPLATE_H_

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "util.h"

using namespace std;
namespace fs = std::filesystem;

class NodeName : public rclcpp::Node {

 public:
  explicit NodeName();
  ~NodeName();

 private:
  // initialization methods
  void InitParams();
  void InitROSEntities();
  void StartThreads();

  // thread functions
  void NativeThreadFunction();
  void TimerThreadFunction();
  void OmpThreadFunction();

  // Callbacks
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu);
  void LiDARCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::CallbackGroup::SharedPtr lidar_cb_group_, imu_cb_group_;
  shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;

  // TF
  shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  shared_ptr<tf2_ros::TransformListener> tf_listener_;
  shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // threading related
  thread native_thread_, omp_thread_;
  atomic<bool> native_thread_running_, timer_thread_running_,
      omp_thread_running_;
  rclcpp::TimerBase::SharedPtr timer_thread_;
  atomic<int> thread_count_;

  // parameters
  string string_param_;
  int int_param_;
  double double_param_;
  bool bool_param_;
  vector<double> vector_param_;
  string lidar_topic_, imu_topic_, trigger_srv_name_;

};  // class NodeName

#endif  // ROS2_NODE_TEMPLATE_H_
