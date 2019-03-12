cmake_minimum_required(VERSION 2.8.3)
project(manip_prelim)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  control_msgs
  genmsg
  geometry_msgs
  hsr_kinematics
  message_generation
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  trajectory_msgs
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_manipulation_msgs
  tmc_planning_msgs
  tmc_utils
)


include_directories(
    ${catkin_INCLUDE_DIRS}
        include
)

add_action_files(
  DIRECTORY action
  FILES
   MoveGripper.action
   pickUp.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
 )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES libvilla_hsrb
  CATKIN_DEPENDS
  actionlib
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  geometry_msgs
  sensor_msgs
  hsr_kinematics
  tmc_eigen_bridge
  tmc_manipulation_types_bridge
  tmc_utils
  actionlib_msgs

)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)



