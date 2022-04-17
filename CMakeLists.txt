cmake_minimum_required(VERSION 3.1.0)
project(ros_lcm_bridge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14") 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(lcm REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
  std_msgs
  tf
  geometry_msgs
  nav_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_gogodog_bridge
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include
)

add_executable(dog_driver_node src/dog_driver_node.cpp src/util.cpp)
target_link_libraries(dog_driver_node ${catkin_LIBRARIES} lcm)

