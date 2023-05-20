cmake_minimum_required(VERSION 3.0.2)
project(autlab4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamic_reconfigure
  effort_controllers
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  joint_state_controller
  joint_state_publisher
  pcl_conversions
  roscpp
  roslint
  pcl_ros
  position_controllers
  robot_state_publisher
  rospy
  sensor_msgs
)

generate_dynamic_reconfigure_options(
   config/Pcl.cfg
)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloud_transformer src/cpp/cloud_transformer.cpp)
add_dependencies(cloud_transformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(cloud_transformer
  ${catkin_LIBRARIES}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
roslint_cpp(src/cpp/cloud_transformer.cpp)
#roslint_cpp(srcipts/template.py)
