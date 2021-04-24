cmake_minimum_required(VERSION 3.0.2)
project(hawkeye)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(gazebo_DIR /usr/lib/x86_64-linux-gnu/cmake/gazebo)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  gazebo
  tf2
  tf2_ros
)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(offboard src/offboard.cpp)
target_link_libraries(offboard
   ${catkin_LIBRARIES}
)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster
 ${catkin_LIBRARIES}
)

add_library(animated_box SHARED src/animated_box.cc)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

add_library(viz_pred_target SHARED src/viz_pred_target.cc)
target_link_libraries(viz_pred_target ${GAZEBO_LIBRARIES})

add_executable(target_gt src/animated_box_gt_pub.cc)
target_link_libraries(target_gt ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)
