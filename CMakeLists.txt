cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rviz_visual_tools
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.8 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs visualization_msgs pcl_ros pcl_conversions
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(detector src/detector_ver2.cpp)
target_link_libraries(detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})

