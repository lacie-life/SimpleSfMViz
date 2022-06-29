cmake_minimum_required(VERSION 3.1)

project(ORB_SLAM3 VERSION 0.1 LANGUAGES CXX)

SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-w)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/CameraModels
    ${CMAKE_SOURCE_DIR}/Thirdparty/g2o
    ${CMAKE_SOURCE_DIR}/Thirdparty/Sophus
    ${CMAKE_SOURCE_DIR}/Thirdparty/DBoW2
)

aux_source_directory(${CMAKE_SOURCE_DIR}/src SRC_FILES)

aux_source_directory(${CMAKE_SOURCE_DIR}/src/CameraModels SRC_SUB_FILES)

add_library(${CMAKE_PROJECT_NAME} SHARED ${SRC_FILES} ${SRC_SUB_FILES})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2_New.so
    ${CMAKE_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)

# RGB-D examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/Examples/RGB-D)

# add_executable(kinect Examples/RGB-D/kinect.cc)

# target_link_libraries(kinect PRIVATE ${CMAKE_PROJECT_NAME})
