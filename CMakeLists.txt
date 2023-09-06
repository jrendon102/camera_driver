cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(
  OpenCV 4 REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_driver
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/camera.cpp
)

add_executable(camera_server_node src/camera_server_node.cpp)
add_executable(camera_client_node src/camera_client_node.cpp)

target_link_libraries(camera_server_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(camera_client_node ${PROJECT_NAME} ${catkin_LIBRARIES})
