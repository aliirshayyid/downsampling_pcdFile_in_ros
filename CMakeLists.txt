cmake_minimum_required(VERSION 2.8.3)
project(pcl_tutorial)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/pcl_tutorial_node.cpp
  src/pcl_tutorial.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
) 


