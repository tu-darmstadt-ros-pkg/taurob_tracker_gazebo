cmake_minimum_required(VERSION 2.8.3)
project(taurob_tracker_hw_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  controller_manager
  gazebo_ros_control
  #pluglinlib
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES taurob_tracker_hw_interface
  CATKIN_DEPENDS  roscpp controller_interface controller_manager gazebo_ros_control #pluglinlib
#  DEPENDS system_lib
)

include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

add_library(taurob_tracker_hw_interface src/robot_hw_sim_base.cpp src/arm_hw_sim.cpp )
target_link_libraries(taurob_tracker_hw_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

