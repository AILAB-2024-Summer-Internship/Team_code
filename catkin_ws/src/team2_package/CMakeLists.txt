cmake_minimum_required(VERSION 3.0.2)
project(team2_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carla_common
  carla_msgs
  carla_spawn_objects
  derived_object_msgs
  geometry_msgs
  nav_msgs
  message_generation
  # python-transform3d-pip
  ros_compatibility
  roscpp
  rosgraph_msgs
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  tf2_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  rviz_visual_tools
)

## Find OpenCV
set(OpenCV_DIR /usr/local/share/OpenCV) # OpenCV 경로 설정
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

find_package(CURL REQUIRED)

## Setup Python packages
catkin_python_setup()

add_message_files(FILES
  globalwaypoints.msg 
  vehicle_state.msg
  longitudinal_controller.msg
  keyboard_msg.msg
  localization_perform_measure.msg
  tracked_object.msg
  tracked_object_array.msg
  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES team2_package
  CATKIN_DEPENDS message_runtime carla_common carla_msgs carla_spawn_objects derived_object_msgs geometry_msgs nav_msgs ros_compatibility roscpp rosgraph_msgs rospy sensor_msgs shape_msgs std_msgs tf2_msgs visualization_msgs
  # DEPENDS system_lib
) # python-transform3d-pip

#NumPy 헤더 파일 경로 추가
execute_process(
  COMMAND python -c "import numpy; print(numpy.get_include())"
  OUTPUT_VARIABLE PYTHON_NUMPY_INCLUDE_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.7 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${PYTHON_NUMPY_INCLUDE_DIR} 
)

add_library(hungarian_lib
  src/perception/Hungarian.cpp
)
target_link_libraries(hungarian_lib ${catkin_LIBRARIES})

#add_executable(controller src/control/controller.cpp)
#target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(pseudo_controller src/control/pseudo_controller.cpp)
add_dependencies(pseudo_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pseudo_controller ${catkin_LIBRARIES})

# add_executable(controller src/control/controller.cpp)
# add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(controller ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(localizer src/perception/localizer.cpp)
add_dependencies(localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(localizer ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(detector src/perception/detector.cpp)
add_dependencies(detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detector ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${PCL_LIBRARIES} ${CURL_LIBRARIES})

add_executable(detector_test src/perception/detector_test.cpp)
add_dependencies(detector_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(detector_test hungarian_lib ${catkin_LIBRARIES} ${PYTHON_LIBRARIES} ${PCL_LIBRARIES} ${CURL_LIBRARIES})

add_executable(gps_to_xyz src/planning/gps_to_xyz.cpp)
add_dependencies(gps_to_xyz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_to_xyz ${catkin_LIBRARIES})

add_executable(global_planner src/planning/global_planner.cpp)
add_dependencies(global_planner team2_package_generate_messages_cpp)
target_link_libraries(global_planner ${catkin_LIBRARIES})

add_executable(behavior_planner src/planning/behavior_planner.cpp)
add_dependencies(behavior_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_planner ${catkin_LIBRARIES})

#add_executable(filter_test src/perception/filter_test.cpp)
#target_link_libraries(filter_test ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
scripts/carla_route_saver.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)