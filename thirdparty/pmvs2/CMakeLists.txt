cmake_minimum_required(VERSION 3.21)
project(pmvs2)

set(CMAKE_CXX_STANDARD 14)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/image/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/numeric/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/pmvs/*.cc)

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/numeric/clapack/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/image/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/numeric/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pmvs/*.h)

add_library(PMVS2 SHARED ${HEADERS} ${SOURCES})

add_executable(pmvs2_demo main.cpp)

target_link_libraries(pmvs2_demo PMVS2)
