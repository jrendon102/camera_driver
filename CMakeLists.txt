cmake_minimum_required(VERSION 3.8)

project(camera_driver VERSION 2.0.1)

find_package(OpenCV 4 REQUIRED)
find_package(OpenMP REQUIRED)

################################################################
# Detect if being built as part of a larger project -------- #
################################################################
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    # Standalone build
    set(ENABLE_CLANG_TIDY ON CACHE BOOL "Enable Clang-Tidy for standalone builds")
    message(STATUS "Building ${PROJECT_NAME} as a standalone library: Clang-Tidy is [${ENABLE_CLANG_TIDY}].")
else()
    # Integrated into a larger project
    set(ENABLE_CLANG_TIDY OFF CACHE BOOL "Disable Clang-Tidy when integrated into larger project")
    message(STATUS "Building ${PROJECT_NAME} as part of a larger project: Clang-Tidy is [${ENABLE_CLANG_TIDY}].")
endif()

################################################################
# C++ Standard ----------------------------------------------- #
################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

################################################################ 
# OPTIONS ---------------------------------------------------- # 
# Enable compiler warnings and optionally treat them as errors #
################################################################
option(ENABLE_WARNINGS "Enable warnings" ON)
option(ENABLE_WARNINGS_AS_ERRORS "Enable warnings as errors" ON)

###############################################################
# Enable Hot Reload for MSVC compilers if supported --------- #
###############################################################
if (POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT 
        "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>"
    )
endif()

################################################################
# Define module path for custom CMake modules and include ---- #
# configuration files ---------------------------------------- #
################################################################
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(GlobalConfig)
include(PackageConfig)
if (ENABLE_CLANG_TIDY)
    include(clang-tidy)
endif()
if(ENABLE_WARNINGS)
    include(Warnings)
endif()

################################################################
# Subdirectories --------------------------------------------- #
################################################################
add_subdirectory(examples)
add_subdirectory(src)