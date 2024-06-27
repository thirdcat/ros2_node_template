set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.8)
project(ros2_node_template)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug build")
  set(CMAKE_CXX_FLAGS "-O2 -pg -ggdb -fPIC" )
else()
  message("Release build")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -w")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_srvs REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(include)

add_executable(ros2_node_template src/main.cpp src/ros2_node_template.cpp)
ament_target_dependencies(ros2_node_template rclcpp tf2 tf2_ros tf2_eigen tf2_geometry_msgs std_msgs std_srvs sensor_msgs geometry_msgs pcl_conversions)
target_link_libraries(ros2_node_template ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)

if (OPENMP_FOUND)
  if (TARGET OpenMP::OpenMP_CXX)
    target_link_libraries(ros2_node_template OpenMP::OpenMP_CXX)
  endif ()
endif ()

install(TARGETS ros2_node_template
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
