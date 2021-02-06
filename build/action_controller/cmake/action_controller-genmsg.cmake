# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "action_controller: 7 messages, 0 services")

set(MSG_I_FLAGS "-Iaction_controller:/home/liu/Multi_Robot_System/src/action_controller/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Itrajectory_msgs:/opt/ros/melodic/share/trajectory_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(action_controller_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" ""
)

get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" "actionlib_msgs/GoalID:action_controller/MultiDofFollowJointTrajectoryResult:actionlib_msgs/GoalStatus:std_msgs/Header"
)

get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" "actionlib_msgs/GoalID:trajectory_msgs/MultiDOFJointTrajectoryPoint:action_controller/MultiDofFollowJointTrajectoryGoal:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Transform:std_msgs/Header:trajectory_msgs/MultiDOFJointTrajectory:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" "actionlib_msgs/GoalID:action_controller/MultiDofFollowJointTrajectoryFeedback:trajectory_msgs/MultiDOFJointTrajectoryPoint:actionlib_msgs/GoalStatus:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Transform:std_msgs/Header:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" "trajectory_msgs/MultiDOFJointTrajectoryPoint:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Transform:std_msgs/Header:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" "trajectory_msgs/MultiDOFJointTrajectoryPoint:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Transform:std_msgs/Header:trajectory_msgs/MultiDOFJointTrajectory:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" NAME_WE)
add_custom_target(_action_controller_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_controller" "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" "actionlib_msgs/GoalID:action_controller/MultiDofFollowJointTrajectoryActionGoal:action_controller/MultiDofFollowJointTrajectoryActionFeedback:trajectory_msgs/MultiDOFJointTrajectoryPoint:action_controller/MultiDofFollowJointTrajectoryGoal:geometry_msgs/Twist:actionlib_msgs/GoalStatus:geometry_msgs/Vector3:action_controller/MultiDofFollowJointTrajectoryResult:geometry_msgs/Transform:std_msgs/Header:action_controller/MultiDofFollowJointTrajectoryActionResult:trajectory_msgs/MultiDOFJointTrajectory:action_controller/MultiDofFollowJointTrajectoryFeedback:geometry_msgs/Quaternion"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)
_generate_msg_cpp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
)

### Generating Services

### Generating Module File
_generate_module_cpp(action_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(action_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(action_controller_generate_messages action_controller_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_cpp _action_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_controller_gencpp)
add_dependencies(action_controller_gencpp action_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_controller_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)
_generate_msg_eus(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
)

### Generating Services

### Generating Module File
_generate_module_eus(action_controller
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(action_controller_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(action_controller_generate_messages action_controller_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_eus _action_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_controller_geneus)
add_dependencies(action_controller_geneus action_controller_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_controller_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)
_generate_msg_lisp(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
)

### Generating Services

### Generating Module File
_generate_module_lisp(action_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(action_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(action_controller_generate_messages action_controller_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_lisp _action_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_controller_genlisp)
add_dependencies(action_controller_genlisp action_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_controller_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)
_generate_msg_nodejs(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
)

### Generating Services

### Generating Module File
_generate_module_nodejs(action_controller
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(action_controller_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(action_controller_generate_messages action_controller_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_nodejs _action_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_controller_gennodejs)
add_dependencies(action_controller_gennodejs action_controller_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_controller_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)
_generate_msg_py(action_controller
  "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectoryPoint.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg;/opt/ros/melodic/share/trajectory_msgs/cmake/../msg/MultiDOFJointTrajectory.msg;/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
)

### Generating Services

### Generating Module File
_generate_module_py(action_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(action_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(action_controller_generate_messages action_controller_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionResult.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryActionFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryGoal.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/liu/Multi_Robot_System/src/action_controller/msg/MultiDofFollowJointTrajectoryAction.msg" NAME_WE)
add_dependencies(action_controller_generate_messages_py _action_controller_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_controller_genpy)
add_dependencies(action_controller_genpy action_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_controller_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(action_controller_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET trajectory_msgs_generate_messages_cpp)
  add_dependencies(action_controller_generate_messages_cpp trajectory_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(action_controller_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_controller
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(action_controller_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET trajectory_msgs_generate_messages_eus)
  add_dependencies(action_controller_generate_messages_eus trajectory_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(action_controller_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(action_controller_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET trajectory_msgs_generate_messages_lisp)
  add_dependencies(action_controller_generate_messages_lisp trajectory_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(action_controller_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_controller
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(action_controller_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET trajectory_msgs_generate_messages_nodejs)
  add_dependencies(action_controller_generate_messages_nodejs trajectory_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(action_controller_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(action_controller_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET trajectory_msgs_generate_messages_py)
  add_dependencies(action_controller_generate_messages_py trajectory_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(action_controller_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
