# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "team2_package: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iteam2_package:/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(team2_package_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" NAME_WE)
add_custom_target(_team2_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team2_package" "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" ""
)

get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" NAME_WE)
add_custom_target(_team2_package_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "team2_package" "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team2_package
)
_generate_msg_cpp(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team2_package
)

### Generating Services

### Generating Module File
_generate_module_cpp(team2_package
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team2_package
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(team2_package_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(team2_package_generate_messages team2_package_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_cpp _team2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_cpp _team2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team2_package_gencpp)
add_dependencies(team2_package_gencpp team2_package_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team2_package_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team2_package
)
_generate_msg_eus(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team2_package
)

### Generating Services

### Generating Module File
_generate_module_eus(team2_package
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team2_package
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(team2_package_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(team2_package_generate_messages team2_package_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_eus _team2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_eus _team2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team2_package_geneus)
add_dependencies(team2_package_geneus team2_package_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team2_package_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team2_package
)
_generate_msg_lisp(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team2_package
)

### Generating Services

### Generating Module File
_generate_module_lisp(team2_package
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team2_package
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(team2_package_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(team2_package_generate_messages team2_package_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_lisp _team2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_lisp _team2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team2_package_genlisp)
add_dependencies(team2_package_genlisp team2_package_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team2_package_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/team2_package
)
_generate_msg_nodejs(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/team2_package
)

### Generating Services

### Generating Module File
_generate_module_nodejs(team2_package
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/team2_package
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(team2_package_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(team2_package_generate_messages team2_package_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_nodejs _team2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_nodejs _team2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team2_package_gennodejs)
add_dependencies(team2_package_gennodejs team2_package_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team2_package_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package
)
_generate_msg_py(team2_package
  "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package
)

### Generating Services

### Generating Module File
_generate_module_py(team2_package
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(team2_package_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(team2_package_generate_messages team2_package_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/longitudinal_controller.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_py _team2_package_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ailab/Desktop/CARLA_Leaderboard_2.0/CARLA_Leaderboard_20/team_code/catkin_ws/src/team2_package/msg/keyboard_msg.msg" NAME_WE)
add_dependencies(team2_package_generate_messages_py _team2_package_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(team2_package_genpy)
add_dependencies(team2_package_genpy team2_package_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS team2_package_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/team2_package
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(team2_package_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/team2_package
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(team2_package_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/team2_package
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(team2_package_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/team2_package)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/team2_package
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(team2_package_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package)
  install(CODE "execute_process(COMMAND \"/home/ailab/anaconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/team2_package
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(team2_package_generate_messages_py std_msgs_generate_messages_py)
endif()
