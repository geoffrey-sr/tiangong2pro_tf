cmake_minimum_required(VERSION 3.5)
project(bodyctrl_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorName.msg"
  "msg/MotorStatus.msg"
  "msg/MotorCtrl.msg"
  "msg/SetMotorPosition.msg"
  "msg/SetMotorSpeed.msg"
  "msg/SetMotorDistance.msg"
  "msg/SetMotorCurTor.msg"
  "msg/CmdMotorCtrl.msg"
  "msg/CmdSetMotorPosition.msg"
  "msg/CmdSetMotorSpeed.msg"
  "msg/CmdSetMotorDistance.msg"
  "msg/CmdSetMotorCurTor.msg"
  "msg/MotorStatusMsg.msg"
  "msg/Euler.msg"
  "msg/Imu.msg"
  "msg/Sri.msg"
  "msg/SriName.msg"
  "msg/MotorStatusMsg2.msg"
  "msg/NodeState.msg"
  "msg/PowerStatus.msg"
  "msg/PowerBoardKeyStatus.msg"
  "msg/TsHandName.msg"
  "msg/SetTsHandPosition.msg"
  "msg/CmdSetTsHandPosition.msg"
  "msg/TsHandStatus.msg"
  "msg/TsHandStatusMsg.msg"
  "msg/SetTsHandCtrl.msg"
  "msg/SetTsHandCtrlItem.msg"
  "msg/CmdSetTsHandCtrl.msg"
  "msg/WaistMotorStatus.msg"
  "msg/CmdSetWaistMotorPos.msg"
  "msg/ExceptionArray.msg"
  "msg/Exception.msg"
  "srv/MotorInit.srv"
  "srv/MotorStart.srv"
  "srv/MotorStop.srv"
  "srv/XSensImuInit.srv"
  "srv/ResetMotorZeroOffset.srv"
  "srv/SetMotorZeroOffset.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

## Generate messages in the 'msg' folder


## Generate services in the 'srv' folder


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

ament_package()
