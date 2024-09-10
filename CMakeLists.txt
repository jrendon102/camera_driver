cmake_minimum_required(VERSION 3.8)

project(camera_driver VERSION 1.0.0)

find_package(OpenCV 4 REQUIRED)
find_package(OpenMP REQUIRED)


## Export package configuration if needed
#include(CMakePackageConfigHelpers)
#write_basic_package_version_file(
#  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#  VERSION ${PROJECT_VERSION}
#  COMPATIBILITY AnyNewerVersion
#)
#
## Configure the package configuration file using Config.cmake.in as a template.
## This file is used by CMake to provide information about how to use the Camera Driver package.
#configure_package_config_file(
#  "${CMAKE_CURRENT_LIST_DIR}/Config.cmake.in"
#  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
#)
#
## Install the generated package configuration files.
## These files will be installed to the specified destination directory
## and can be used by other projects to find and use Camera Driver as a dependency.
#install(
#  FILES
#  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
#  DESTINATION lib/cmake/${PROJECT_NAME}
#)
#
## Create an uninstall target
#if(NOT TARGET uninstall)
#  configure_file(
#    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeUninstall.cmake.in"
#    "${CMAKE_CURRENT_BINARY_DIR}/CMakeUninstall.cmake"
#    IMMEDIATE @ONLY)
#
#  add_custom_target(uninstall
#    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/CMakeUninstall.cmake)
#endif()

# Enable Hot Reload for MSVC compilers if supported
if (POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT 
        "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>"
    )
endif()

# Define module path for custom CMake modules and include configuration files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(GlobalConfig)

add_subdirectory(src)