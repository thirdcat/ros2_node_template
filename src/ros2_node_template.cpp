#include "ros2_node_template/ros2_node_template.h"

#include <sstream>

NodeName::NodeName() : Node("node_name") {

  RCLCPP_INFO(this->get_logger(), "node_name node is created");
  InitParams();
  InitROSEntities();
  StartThreads();

  return;
}

NodeName::~NodeName() {
  native_thread_running_ = false;

  if (native_thread_.joinable())
    native_thread_.join();

  omp_thread_running_ = false;

  if (omp_thread_.joinable())
    omp_thread_.join();

  timer_thread_running_ = false;

  return;
}

void NodeName::InitParams() {
  RCLCPP_INFO(this->get_logger(), "Initialize the parameters");

  string_param_ = this->declare_parameter<string>("string_param", "default");
  int_param_ = this->declare_parameter<int>("int_param", -1);
  double_param_ = this->declare_parameter<double>("double_param", -1.0);
  bool_param_ = this->declare_parameter<bool>("bool_param", false);
  vector_param_ = this->declare_parameter<vector<double>>("vector_param",
                                                          vector<double>({}));

  lidar_topic_ = this->declare_parameter<string>("lidar_topic", "/lidar");
  imu_topic_ = this->declare_parameter<string>("imu_topic", "/imu");
  trigger_srv_name_ =
      this->declare_parameter<string>("trigger_service_name", "/default");

  RCLCPP_INFO(this->get_logger(),
              "Node Parameters: string_param: %s, "
              "int_param: %d, "
              "double_param: %.3f, "
              "bool_param: %s, "
              "lidar_topic: %s, "
              "imu_topic: %s, "
              "trigger_service_name: %s",
              string_param_.c_str(), int_param_, double_param_,
              bool_param_ ? "true" : "false", lidar_topic_.c_str(),
              imu_topic_.c_str(), trigger_srv_name_.c_str());

  if (!vector_param_.empty()) {
    stringstream ss;
    for (double v : vector_param_)
      ss << v << " ";
    RCLCPP_INFO(this->get_logger(), "vector_param: %s", ss.str().c_str());
  }

  return;
}

void NodeName::InitROSEntities() {
  RCLCPP_INFO(this->get_logger(), "Register the publishers & subscribers");

  // TF instances
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Publisher
  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/output_pcd", rclcpp::SensorDataQoS());

  // LiDAR & IMU subscribers with separate callback groups
  lidar_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = lidar_cb_group_;
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      bind(&NodeName::LiDARCallback, this, std::placeholders::_1),
      lidar_sub_opt);

  imu_cb_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = imu_cb_group_;
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      bind(&NodeName::IMUCallback, this, std::placeholders::_1), imu_sub_opt);

  // Parameter event handler
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  cb_handle_ = param_subscriber_->add_parameter_event_callback(
      [this](const rcl_interfaces::msg::ParameterEvent event) {
        auto params = event.changed_parameters;
        for (auto& param : params) {
          if (param.name == "string_param") {
            string_param_ = param.value.string_value;
            RCLCPP_INFO(this->get_logger(), "string_param is changed to %s",
                        string_param_.c_str());
          } else if (param.name == "int_param") {
            int_param_ = param.value.integer_value;
            RCLCPP_INFO(this->get_logger(), "int_param is changed to %d",
                        int_param_);
          } else if (param.name == "double_param") {
            double_param_ = param.value.double_value;
            RCLCPP_INFO(this->get_logger(), "double_param is changed to %.3f",
                        double_param_);
          } else if (param.name == "bool_param") {
            bool_param_ = param.value.bool_value;
            RCLCPP_INFO(this->get_logger(), "bool_param is changed to %s",
                        bool_param_ ? "true" : "false");
          } else if (param.name == "vector_param") {
            vector_param_ = param.value.double_array_value;
            stringstream ss;
            for (double v : vector_param_)
              ss << v << " ";
            RCLCPP_INFO(this->get_logger(), "vector_param is changed to %s",
                        ss.str().c_str());
          }
        }
      });

  // Trigger service with function pointer
  trigger_srv_ = this->create_service<std_srvs::srv::Trigger>(
      trigger_srv_name_,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Trigger service is called");
        response->success = true;
        response->message = "Trigger service is called";
      });

  return;
}

void NodeName::StartThreads() {
  RCLCPP_INFO(this->get_logger(), "Start the c++ native & ROS 2 timer threads");
  thread_count_ = 0;
  native_thread_running_ = true;
  native_thread_ = thread(&NodeName::NativeThreadFunction, this);

  timer_thread_running_ = true;
  timer_thread_ =
      this->create_wall_timer(std::chrono::milliseconds(1),
                              std::bind(&NodeName::TimerThreadFunction, this));

  omp_thread_running_ = true;
  omp_thread_ = thread(&NodeName::OmpThreadFunction, this);

  return;
}

void NodeName::LiDARCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  static unsigned int count = 0;

  if (count % 10 == 0)
    RCLCPP_INFO(this->get_logger(), "Got %d LiDAR msg from %s topic", count,
                lidar_topic_.c_str());

  count++;

  // Publish the received LiDAR data after changing the frame_id
  sensor_msgs::msg::PointCloud2 output_cloud(*cloud);
  output_cloud.header.frame_id = "map";

  // TF2 usage example, map -> cloud frame TF as identity, now this pcd will be shown in RViz
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = cloud->header.stamp;
  tf.header.frame_id = "map";
  tf.child_frame_id = cloud->header.frame_id;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;
  tf.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(tf);

  pcd_pub_->publish(output_cloud);
}

void NodeName::IMUCallback(const sensor_msgs::msg::Imu::SharedPtr imu) {
  static unsigned int count = 0;

  if (count % 200 == 0)
    RCLCPP_INFO(this->get_logger(), "Got %d IMU msg from %s topic", count,
                imu_topic_.c_str());

  count++;
}

void NodeName::NativeThreadFunction() {
  RCLCPP_INFO(this->get_logger(), "Native thread started");

  while (native_thread_running_) {
    thread_count_++;
    int loaded_count = thread_count_.load();

    if (loaded_count % 10000 == 0) {
      RCLCPP_INFO(
          this->get_logger(),
          "Native thread count is %d, this should be printed per 5 sec.",
          loaded_count);
    }

    this_thread::sleep_for(chrono::milliseconds(1));
  }

  RCLCPP_INFO(this->get_logger(), "Native thread stopped");
  return;
}

void NodeName::TimerThreadFunction() {
  if (!timer_thread_running_)
    return;

  thread_count_++;
  int loaded_count = thread_count_.load();

  if (loaded_count % 10000 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "Timer thread count is %d, this should be printed per 5 sec.",
                loaded_count);
  }
  return;
}

void NodeName::OmpThreadFunction() {
  RCLCPP_INFO(this->get_logger(), "OpenMP thread started");

  unsigned int count = 0;
  auto t0 = chrono::high_resolution_clock::now();

  for (int i = 0; i < 100000; i++) {
    if (is_prime(i))
      count++;
  }

  auto t1 = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
  RCLCPP_INFO(this->get_logger(),
              "non-OpenMP counting primes for 1e5 took %d us, count: %d",
              duration, count);

  count = 0;

  t0 = chrono::high_resolution_clock::now();

#pragma omp parallel for reduction(+ : count)
  for (int i = 0; i < 100000; i++) {

#pragma omp nowait
    if (is_prime(i)) {
      count++;
    }
  }

  t1 = chrono::high_resolution_clock::now();
  duration = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
  RCLCPP_INFO(this->get_logger(),
              "OpenMP counting primes for 1e5 took %d us, count: %d", duration,
              count);

  RCLCPP_INFO(this->get_logger(), "OpenMP thread stopped");
  return;
}
