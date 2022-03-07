cmake_minimum_required(VERSION 3.0)
project(pmvs2)

set(CMAKE_CXX_STANDARD 14)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(GSL REQUIRED)

set(CUSTOM_JPEG_PATH "" CACHE STRING "path to custom installation of libjpeg")
if(CUSTOM_JPEG_PATH)
        find_package(JPEG REQUIRED 
            PATHS ${CUSTOM_JPEG_PATH} 
            NO_SYSTEM_ENVIRONMENT_PATH
        )
else()
        find_package(JPEG REQUIRED) #default search
endif()

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/image/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/image/jpeg/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/numeric/lmmin/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/numeric/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/pmvs/*.cc)

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/numeric/clapack/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/numeric/lmmin/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/image/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/image/jpeg/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/numeric/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pmvs/*.h)

#add_subdirectory(image/jpeg)

add_library(PMVS2 SHARED ${HEADERS} ${SOURCES})

target_link_libraries(PMVS2 GSL::gsl GSL::gslcblas -lpthread -llapack /usr/lib/x86_64-linux-gnu/libjpeg.so)

add_executable(pmvs2_demo main.cpp)

target_link_libraries(pmvs2_demo PMVS2)
