cmake_minimum_required(VERSION 2.8.3)
project(obstacle_runout)

SET(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

include_directories(
  include
	${catkin_INCLUDE_DIRS}) 

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp std_msgs sensor_msgs pcl_ros
  INCLUDE_DIRS include
)


add_executable(DstarMain src/Dstar.cpp src/DstarMain.cpp src/tool/DstarTool.cpp src/tool/common.cpp src/tool/B_Spline.cpp)
target_link_libraries(DstarMain ${catkin_LIBRARIES})

add_executable(read_map src/read_map.cpp src/tool/readMap.cpp src/tool/read.cpp src/tool/readYaml.cpp
                src/tool/createGrid.cpp src/tool/obstacleAdd.cpp)
target_link_libraries(read_map ${catkin_LIBRARIES})
