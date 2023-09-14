cmake_minimum_required(VERSION 3.0.2)
project(camera_driver)

find_package(
  OpenCV 4 REQUIRED
)

include_directories(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/camera.cpp
  src/camera_utils.cpp
)

add_executable(camera_test src/camera_test.cpp)
target_link_libraries(camera_test ${PROJECT_NAME} ${OpenCV_LIBS})