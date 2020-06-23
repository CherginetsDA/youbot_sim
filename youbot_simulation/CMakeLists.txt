cmake_minimum_required(VERSION 2.8.3)
project(youbot_simulation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(RL COMPONENTS MDL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_state_publisher
  urdf
  trajectory_msgs
  brics_actuator
  nav_msgs
  pr2_controller_interface
  control_toolbox
  pr2_controllers_msgs
  realtime_tools
  orocos_kdl
  sensor_msgs
  control_msgs
  actionlib
  pr2_gazebo_plugins
)

find_package (Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp robot_state_publisher urdf trajectory_msgs brics_actuator nav_msgs pr2_controller_interface control_toolbox pr2_controllers_msgs realtime_tools sensor_msgs control_msgs actionlib pr2_gazebo_plugins
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(youbot_gazebo_message_wrapper src/youbot_gazebo_msg_wrapper.cpp)
target_link_libraries(youbot_gazebo_message_wrapper
  ${catkin_LIBRARIES}
)

add_executable(home src/home.cpp)
target_link_libraries(home ${catkin_LIBRARIES})
