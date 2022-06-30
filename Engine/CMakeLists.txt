cmake_minimum_required(VERSION 3.2)

project(GreenHouseAR LANGUAGES CXX)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

source_group("Header Files" FILES ${HEADERS})
source_group("Source Files" FILES ${SOURCES})

set(UI ${CMAKE_CURRENT_LIST_DIR}/ui/mainwindow.ui
       ${CMAKE_CURRENT_LIST_DIR}/ui/qconfigdialog.ui
       ${CMAKE_CURRENT_LIST_DIR}/ui/qhelpdialog.ui)

qt5_wrap_ui(UI_HEADERS ${UI})

list(APPEND RESOURCES ${CMAKE_CURRENT_LIST_DIR}/resources.qrc)

qt5_add_resources(RCC_SOURCES ${RESOURCES})

add_executable(GreenHouseAR
    ${HEADERS}
    ${SOURCES}
    ${UI_HEADERS}
    ${RCC_SOURCES}
)

target_link_libraries(GreenHouseAR PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)


target_link_libraries(GreenHouseAR PRIVATE
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
)



