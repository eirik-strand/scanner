cmake_minimum_required(VERSION 2.8.3)
project(scanner)

add_compile_options(-std=c++11)



cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

SET(OpenCV_DIR /home/e/opencv)



find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} )

add_executable(scanner src/scanner.cpp include/scanner.h)



target_link_libraries(scanner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})




#############
## Install ##not found_pub
#############

install(TARGETS scanner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
