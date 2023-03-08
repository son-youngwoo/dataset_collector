cmake_minimum_required(VERSION 3.5.1)
project(dataset_collector)

set(CMAKE_CXX_STANDARD 11)


#set(PKG_DEPS
#  roscpp
#  roslib
#  std_msgs)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS} 
  std_msgs
  gazebo_msgs
  message_generation
  std_srvs
  nav_msgs   
  geometry_msgs
  aidin_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_octomap
  grid_map_cv
  cv_bridge
  eigen_conversions
  sensor_msgs)

find_package(Eigen3 REQUIRED
)
#find_package(OpenCV 3 REQUIRED)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

add_message_files(
  FILES
  dataset.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  grid_map_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIR}  
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dataset_collector
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
#  LIBRARIES simple_class_node
#  CATKIN_DEPENDS ${PKG_DEPS}
#    grid_map_core
#    grid_map_ros
#    grid_map_msgs
#    eigen_conversions
#  DEPENDS
)

#add_library(thread_class_node
#  src/thread_node_class.cpp)
add_executable(dataset_collector src/dataset_collector.cpp)
target_link_libraries(dataset_collector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(respawn_robot src/respawn_robot.cpp)
add_dependencies(respawn_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(respawn_robot
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS src/FH_ROS.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)






