cmake_minimum_required(VERSION 3.0)

find_package(OpenCV REQUIRED)

project(yolov4 VERSION 1.0)

set(CMAKE_BUILD_TYPE "Release")

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(/home/csl/TempWork/rgbd-slam/yolov4-learn/lib)

link_libraries("/home/csl/TempWork/rgbd-slam/yolov4-learn/lib/libdarknet.so")

include_directories(./include)

aux_source_directory(./src SRC_DIR)

add_executable(yolov4 main.cpp ${SRC_DIR})

target_link_libraries(yolov4 ${OpenCV_LIBS})
