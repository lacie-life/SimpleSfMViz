cmake_minimum_required(VERSION 3.2)

project(GreenHouseAR LANGUAGES CXX)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

find_package(OpenCV 4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL)
find_package(Boost COMPONENTS system filesystem REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/../Slam
    ${CMAKE_SOURCE_DIR}/../Slam/include
    ${CMAKE_SOURCE_DIR}/../Slam/include/CameraModels
    ${CMAKE_SOURCE_DIR}/../Slam/Thirdparty/g2o
    ${CMAKE_SOURCE_DIR}/../Slam/Thirdparty/Sophus
    ${CMAKE_SOURCE_DIR}/../Slam/Thirdparty/DBoW2

    /usr/local/include
    /usr/include/gtk-2.0
    /usr/lib/x86_64-linux-gnu/gtk-2.0/include
    /usr/include/pango-1.0
    /usr/include/atk-1.0
    /usr/include/gdk-pixbuf-2.0
    /usr/include/libmount
    /usr/include/blkid
    /usr/include/pango-1.0
    /usr/include/fribidi
    /usr/include/cairo
    /usr/include/pixman-1
    /usr/include/harfbuzz
    /usr/include/glib-2.0
    /usr/lib/x86_64-linux-gnu/glib-2.0/include
    /usr/include/uuid
    /usr/include/freetype2
    /usr/include/libpng16
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIR})

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

target_link_libraries(GreenHouseAR PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CMAKE_SOURCE_DIR}/../Slam/lib/libORB_SLAM3.so
    ${CMAKE_SOURCE_DIR}/../Slam/Thirdparty/DBoW2/lib/libDBoW2_New.so
    ${CMAKE_SOURCE_DIR}/../Slam/Thirdparty/g2o/lib/libg2o.so
    ${CMAKE_SOURCE_DIR}/../Model/yolov4-learn/lib/libdarknet.so
    -lboost_serialization
    -lcrypto
    pthread
    gtk-x11-2.0
    gdk-x11-2.0
    pangocairo-1.0
    atk-1.0
    cairo
    gdk_pixbuf-2.0
    gio-2.0
    pangoft2-1.0
    pango-1.0
    gobject-2.0
    glib-2.0
    harfbuzz
    fontconfig
    freetype
)


target_link_libraries(GreenHouseAR PRIVATE
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)



