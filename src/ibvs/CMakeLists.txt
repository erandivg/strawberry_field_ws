cmake_minimum_required(VERSION 3.0.2)
project(ibvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge  
  image_transport
  visp_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ibvs
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport cv_bridge visp_ros
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${VISP_INCLUDE_DIRS})

add_executable(homography_vision src/homography_vision.cpp src/vision_utils.cpp)
target_link_libraries(homography_vision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(homography_control src/homography_control.cpp)
target_link_libraries(homography_control
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${VISP_LIBRARIES}
)

