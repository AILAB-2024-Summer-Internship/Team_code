execute_process(COMMAND "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
