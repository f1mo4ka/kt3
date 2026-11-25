cmake_minimum_required(VERSION 3.0.2)
project(lab_elchukova)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission src/mission.cpp)
target_link_libraries(mission ${catkin_LIBRARIES})
