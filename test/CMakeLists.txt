cmake_minimum_required(VERSION 3.10)

project(test)

add_definitions(-w)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/include
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/include/CameraModels
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/Sophus
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2
)

add_executable(${CMAKE_PROJECT_NAME} kinect.cc)

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/lib/libORB_SLAM3.so
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2_New.so
    ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
    -lboost_serialization
    -lcrypto
)
