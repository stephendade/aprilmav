# C Library for using the Jetson PVA interface for
# detecting Apriltags
#

cmake_minimum_required(VERSION 3.5)

project(apriltagPVA VERSION 0.1.0 DESCRIPTION "PVA accelerated Apriltag detection")

set(CMAKE_CXX_STANDARD 11)

find_package(vpi 3.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

if(OpenCV_VERSION VERSION_LESS 2.4.10)
    message(FATAL_ERROR "OpenCV version must be at least 2.4.10")
endif()

# Main library
pybind11_add_module(${PROJECT_NAME}
    apriltags-vpi.cpp
    apriltags-vpi-pywrap.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC vpi opencv_core ${Python3_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})

# Copy to correct dir
add_custom_command(
    TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ../../modules
)