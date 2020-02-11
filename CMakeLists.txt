cmake_minimum_required(VERSION 2.8.3)
project(lfd_experiments)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  allegro_hand_kdl
  dmp_tools
  kdl_control_tools
  roscpp
  spring_framework
  kdl_conversions
  moveit_ros_planning_interface
  arq_ur5
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Boost REQUIRED)

# For YouCompleteMe
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  TrajectoryRequest.srv
  SpringUpdate.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES lfd_experiments
 CATKIN_DEPENDS
  allegro_hand_kdl
  dmp_tools
  kdl_control_tools
  roscpp
  spring_framework
  kdl_conversions
  moveit_ros_planning_interface
  arq_ur5
  std_msgs
  message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/lfd_experiments.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare C++ executables
add_executable(record_demo src/record_demo.cpp
  src/frame_state_listener.cpp
)

add_executable(joint_trajectory_server src/joint_trajectory_server.cpp)

add_executable(task_trajectory_server src/task_trajectory_server.cpp)

add_executable(spring_trajectory_server src/spring_trajectory_server.cpp)

add_executable(envelop_spring_node src/envelop_spring_node.cpp)

add_executable(grasp_node src/grasp_node.cpp)

add_executable(tactile_stop src/tactile_stop_node.cpp)

add_executable(adjust_stiffness src/adjust_stiffness_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(joint_trajectory_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(record_demo
  ${catkin_LIBRARIES}
)

target_link_libraries(joint_trajectory_server
  ${catkin_LIBRARIES}
)

target_link_libraries(task_trajectory_server
  ${catkin_LIBRARIES}
)

target_link_libraries(spring_trajectory_server
  ${catkin_LIBRARIES}
)

target_link_libraries(envelop_spring_node
  ${catkin_LIBRARIES}
)

target_link_libraries(grasp_node
  ${catkin_LIBRARIES}
)

target_link_libraries(tactile_stop
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(adjust_stiffness
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
