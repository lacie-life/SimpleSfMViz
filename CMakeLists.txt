cmake_minimum_required(VERSION 3.21)
project(GreenHouseAR)

set(CMAKE_CXX_STANDARD 11)
SET(COMPILE_FLAGS "-m64 -std=c++14 -Wall -Wextra -pedantic")
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${COMPILE_FLAGS}" )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

list(APPEND QML_IMPORT_PATH "/home/lacie/Qt/5.15.2/gcc_64/qml")
set(CMAKE_PREFIX_PATH "/home/lacie/Qt/5.15.2/gcc_64")

set(QML_IMPORT_PATH ${QML_IMPORT_PATH}
    CACHE STRING "Qt Creator 4.1 extra qml import paths"
    FORCE
)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

set(GLEW_USE_STATIC_LIBS OFF)
add_definitions( -DGLEW_STATIC )

set(WITH_PCL ON  CACHE BOOL "Enable pcl")
set(WITH_OPENGL OFF CACHE BOOL "Enable OpenGL")

set(GLEW_VERBOSE ON)
include(FindGLEW)
find_package(GLEW REQUIRED)
if(GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIR})
    link_libraries(${GLEW_LIBRARIES})
endif(GLEW_FOUND)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

if(OpenGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIR})
endif(OpenGL_FOUND)

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})

find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)

find_package(Qt5Qml REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Gui REQUIRED)

find_package(Qt53DInput REQUIRED)
find_package(Qt53DQuick REQUIRED)
find_package(Qt53DRender REQUIRED)
find_package(Qt53DQuickRender REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS} ${QtQml_INCLUDE_DIRS})
add_definitions(${Qt5Widgets_DEFINITIONS} ${QtQml_DEFINITIONS} ${${Qt5Quick_DEFINITIONS}})

# PCL Library
find_package(PCL REQUIRED COMPONENTS common io)
link_directories(${PCL_LIBRARY_DIRS})

include_directories(include)

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/openGL/*.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/include/pcl/*.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/openGL/*.cpp
                  ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl/*.cpp
                  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

qt5_add_resources(QT_RESOURCES ${CMAKE_CURRENT_SOURCE_DIR}/assest/qml.qrc)

add_executable(${PROJECT_NAME}
                ${HEADERS}
                ${SOURCES}
                ${QT_RESOURCES})

target_compile_definitions(${PROJECT_NAME}
                PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)


target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARY} glfw GLEW::GLEW Qt5::Core Qt5::Quick Qt5::Qml Qt5::Widgets ${PCL_LIBRARIES} stdc++)

target_include_directories(${PROJECT_NAME} PRIVATE PRIVATE ${PCL_INCLUDE_DIRS})
#target_compile_definitions(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS}) causes command-line error in cmake from ubuntu 16.04
target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_PCL=1)

qt5_use_modules(${PROJECT_NAME} Qml Widgets Quick 3DCore 3DQuick 3DRender 3DQuickRender)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/assest
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

 execute_process(
   COMMAND "${CMAKE_COMMAND}" "-E" "create_symlink" ${CMAKE_CURRENT_SOURCE_DIR}/assest/data ${CMAKE_CURRENT_BINARY_DIR}/assest/data
 )
