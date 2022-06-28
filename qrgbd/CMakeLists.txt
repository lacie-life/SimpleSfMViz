cmake_minimum_required(VERSION 3.5)

project(qrgbd VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

# customer ---

# add resource file
file(GLOB_RECURSE QRC_SOURCE_FILES ${CMAKE_SOURCE_DIR}/resource.qrc)

qt6_add_resources(QRC_FILES ${QRC_SOURCE_FILES})

source_group("Resource Files" FILES ${QRC_SOURCE_FILES})

add_definitions(-w)

find_package(OpenCV 4.5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/include
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/include/CameraModels
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/Sophus
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2

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
add_definitions(${PCL_DEFINITIONS})
# customer ---

set(PROJECT_SOURCES
  main.cpp
  mainwindow.cpp
  mainwindow.h
  mainwindow.ui
  # customer ---

  helpdialog.cpp
  helpdialog.h
  helpdialog.ui

  configdialog.cpp
  configdialog.h
  configdialog.ui

  slam.cpp
  slam.h

  rebuilder.h
  rebuilder.cpp

  recognizer.h
  recognizer.cpp

  timer.h

  colorPrj.h
  colorPrj.cpp

  yolo.h
  yolo_v2_class.hpp
  yolo.cpp


  csv.h

  embed.cpp
  embed.h

  optimize.h
  optimize.cpp

  pcl_visual.cpp
  pcl_visual.h

  ${QRC_FILES}

  # customer ---
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qrgbd
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET qrgbd APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(qrgbd SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(qrgbd
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(qrgbd PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# customer ---

target_link_libraries(qrgbd PRIVATE
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/lib/libORB_SLAM3.so
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2_New.so
  ${CMAKE_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
  -lboost_serialization
  -lcrypto
  ${CMAKE_SOURCE_DIR}/../yolov4-learn/lib/libdarknet.so

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
# customer ---

set_target_properties(qrgbd PROPERTIES
    MACOSX_BUNDLE_GUI_IDENTIFIER my.example.com
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(qrgbd)
endif()
