cmake_minimum_required(VERSION 3.0.2)
project(camera_usb_driver)

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
  LIBRARIES camera_usb_driver
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

add_executable(send_video_feeds src/send_video_feeds.cpp)
add_executable(view_video_feeds src/view_video_feeds.cpp)

target_link_libraries(send_video_feeds ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(view_video_feeds ${PROJECT_NAME} ${catkin_LIBRARIES})
