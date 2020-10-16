cmake_minimum_required(VERSION 2.8.3)
project(aerial_manipulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  mav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  cmake_modules

)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} /usr/include/eigen3)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

add_library(control_plugin SHARED src/controller.cpp)
target_link_libraries(control_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(kinova_tele src/robot_teleop_node.cpp)
target_link_libraries(kinova_tele ${catkin_LIBRARIES} gazebo_math gazebo_common)

add_executable(harrier_tele src/uav_teleop_node.cpp)
target_link_libraries(harrier_tele ${catkin_LIBRARIES} )



