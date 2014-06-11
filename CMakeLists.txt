cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(Faces)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB src ./src/*.cpp)

add_executable (face ${src})
target_link_libraries (face ${PCL_LIBRARIES})
