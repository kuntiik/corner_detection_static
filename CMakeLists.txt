cmake_minimum_required(VERSION 3.0)
project(playground)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(functions)

add_executable(test_file test_file.cpp)
add_executable(convert_cloud convert_cloud.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
target_link_libraries(convert_cloud ${OpenCV_LIBS} ${PCL_LIBRARIES} pcl_functions)

#set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

