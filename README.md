# ROS 2 template node 

this ROS 2 template node includes the following features

**Features**

* ROS 2 native launch & param config examples
* sensor callbacks
* service callbacks
* real-time param callbacks
* three threading examples (rclcpp timer & C++ native & OMP example)
* Google clang-format & .gitignore deployed
* Eigen & TF2 using util codes (include/util.h) 

## Preliminaries

* ROS 2 (humble)
* dependencies listed below

## Usage

$ ./convert_names [package name] [node name]
$ git init

## Dependencies on External Libraries

You can exclude the dependencies listed below from the CMakeLists.txt if unnecessary

* Eigen
* Ceres Solver
* PCL
* Open3D
* Boost
* TBB
* OpenCV
* Glog
* OpenMP
