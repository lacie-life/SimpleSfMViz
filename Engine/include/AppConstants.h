#ifndef APPCONSTANTS_H
#define APPCONSTANTS_H

#include <QObject>
#include <QString>
#include <QUrl>
#include <QColor>
#include <QMutex>
#include <QCoreApplication>

#ifndef MACRO_DEFINE
#define MACRO_DEFINE

#define CONSOLE qDebug() << "[" << __FUNCTION__ << "] "

#endif

#ifndef BUILD_DIR

#define BUILD_DIR QCoreApplication::applicationDirPath()

#endif

// Dataset
#define SEQ_PATH "/home/jun/Github/Data/SLAM-Dataset/rgbd_dataset_freiburg3_long_office_household"
#define RGB_IMAGE_PATH "/home/jun/Github/Data/SLAM-Dataset/rgbd_dataset_freiburg3_long_office_household/rgb"
#define DEPTH_IMAGE_PATH "/home/jun/Github/Data/SLAM-Dataset/rgbd_dataset_freiburg3_long_office_household/depth"
#define ASSO_DEPTH_PATH "/home/jun/Github/Data/SLAM-Dataset/rgbd_dataset_freiburg3_long_office_household/depth.txt"
#define ASSO_RGB_PATH "/home/jun/Github/Data/SLAM-Dataset/rgbd_dataset_freiburg3_long_office_household/rgn.txt"

// SLAM System
#define SETTING_PATH "data/config/rgbd.yaml"
#define VOCABULARY_PATH "data/ORBvoc.txt"

// Recognizer
#define CLASSES_PATH "data/model/coco.names"
#define MODEL_CONFIG "data/model/yolov4.cfg"
#define MODEL_WEIGHTS "data/model/yolov4.weights"

// GUI
#define BACKGOUND ":/images/data/res/background.jpg"
#define APP_ICON ":/images/data/res/icon.png"

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#endif // APPCONSTANTS_H