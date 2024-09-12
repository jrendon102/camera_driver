cmake_minimum_required(VERSION 3.8)

project(camera_driver VERSION 2.0.0)

find_package(OpenCV 4 REQUIRED)
find_package(OpenMP REQUIRED)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

# OPTIONS
# Enable compiler warnings and optionally treat them as errors
option(ENABLE_WARNINGS "Enable warnings" ON)
option(ENABLE_WARNINGS_AS_ERRORS "Enable warnings as errors" ON)

# Enable Hot Reload for MSVC compilers if supported
if (POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT 
        "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>"
    )
endif()

# Define module path for custom CMake modules and include configuration files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
include(clang-tidy)
include(GlobalConfig)
include(PackageConfig)
if(ENABLE_WARNINGS)
    include(Warnings)
endif()

add_subdirectory(examples)
add_subdirectory(src)