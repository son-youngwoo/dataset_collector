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
  sensor_msgs
  )

find_package(Eigen3 REQUIRED
)
#find_package(OpenCV 3 REQUIRED)

## Gazebo
find_package(gazebo REQUIRED)

find_package(OpenCV REQUIRED
  COMPONENTS
  opencv_highgui
  CONFIG
)

add_message_files(
  FILES
  dataset.msg
  dataset_rec_model.msg
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

  # Gazebo
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
    # Gazebo
    ${GAZEBO_LIBRARY_DIRS}
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

add_executable(dataset_collector_rec_model src/dataset_collector_rec_model.cpp)
target_link_libraries(dataset_collector_rec_model ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(get_global_map src/get_global_map.cpp)
target_link_libraries(get_global_map ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(respawn_robot src/respawn_robot.cpp)
add_dependencies(respawn_robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(respawn_robot
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(respawn_rec_model src/respawn_rec_model.cpp)
add_dependencies(respawn_rec_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(respawn_rec_model
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(respawn_rec_model_base src/respawn_rec_model_base.cpp)
add_dependencies(respawn_rec_model_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(respawn_rec_model_base
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


# add_library(rec_model_plugin src/rec_model_plugin.cpp)
# target_link_libraries(rec_model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(rec_model_plugin ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/FH_ROS.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/dataset_checker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/dataset_merge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/image_pixel_changer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/image_size_changer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)





