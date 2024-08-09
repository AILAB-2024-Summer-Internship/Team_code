# Install script for directory: /home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team2_package/msg" TYPE FILE FILES
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/globalwaypoints.msg"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/vehicle_state.msg"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/localization_perform_measure.msg"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/tracked_object.msg"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/tracked_object_array.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team2_package/cmake" TYPE FILE FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/installspace/team2_package-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/include/team2_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/share/roseus/ros/team2_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/share/common-lisp/ros/team2_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/share/gennodejs/ros/team2_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/ailab/anaconda3/envs/py37/bin/python3" -m compileall "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/lib/python3/dist-packages/team2_package")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/lib/python3/dist-packages/team2_package" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/lib/python3/dist-packages/team2_package" FILES_MATCHING REGEX "/home/ailab/Desktop/CARLA_Leaderboard_2\\.0/CARLA_Leaderboard_20/team_code/catkin_ws/devel/lib/python3/dist-packages/team2_package/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/installspace/team2_package.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team2_package/cmake" TYPE FILE FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/installspace/team2_package-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team2_package/cmake" TYPE FILE FILES
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/installspace/team2_packageConfig.cmake"
    "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/installspace/team2_packageConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team2_package" TYPE FILE FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/team2_package" TYPE PROGRAM FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/build/team2_package/catkin_generated/installspace/carla_route_saver.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/team2_package" TYPE DIRECTORY FILES "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/launch")
endif()

