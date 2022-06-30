/********************************************************************************
** Form generated from reading UI file 'qhelpdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QHELPDIALOG_H
#define UI_QHELPDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBox>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_QHelpDialog
{
public:
    QGridLayout *gridLayout;
    QPushButton *btn_ok;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *horizontalSpacer_2;
    QFrame *frame;
    QGridLayout *gridLayout_2;
    QToolBox *toolBox;
    QWidget *main;
    QGridLayout *gridLayout_6;
    QHBoxLayout *horizontalLayout;
    QLabel *label_Q;
    QLabel *label_R;
    QLabel *label_G;
    QLabel *label_B;
    QLabel *label_D;
    QWidget *overview;
    QGridLayout *gridLayout_3;
    QTextBrowser *textBrowser;
    QWidget *authors;
    QGridLayout *gridLayout_4;
    QTextBrowser *textBrowser_2;
    QWidget *details;
    QGridLayout *gridLayout_5;
    QTextBrowser *textBrowser_3;

    void setupUi(QDialog *QHelpDialog)
    {
        if (QHelpDialog->objectName().isEmpty())
            QHelpDialog->setObjectName(QString::fromUtf8("QHelpDialog"));
        QHelpDialog->resize(875, 609);
        gridLayout = new QGridLayout(QHelpDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        btn_ok = new QPushButton(QHelpDialog);
        btn_ok->setObjectName(QString::fromUtf8("btn_ok"));
        btn_ok->setMinimumSize(QSize(100, 30));
        btn_ok->setMaximumSize(QSize(200, 16777215));
        QFont font;
        font.setFamily(QString::fromUtf8("Ubuntu Mono"));
        font.setPointSize(14);
        font.setBold(false);
        font.setItalic(true);
        btn_ok->setFont(font);

        gridLayout->addWidget(btn_ok, 1, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 0, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Fixed, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer_2, 1, 2, 1, 1);

        frame = new QFrame(QHelpDialog);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setFont(font);
        frame->setFrameShape(QFrame::NoFrame);
        frame->setFrameShadow(QFrame::Raised);
        gridLayout_2 = new QGridLayout(frame);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        toolBox = new QToolBox(frame);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Ubuntu Mono"));
        font1.setPointSize(14);
        font1.setBold(true);
        font1.setItalic(true);
        toolBox->setFont(font1);
        toolBox->setFrameShape(QFrame::NoFrame);
        toolBox->setLineWidth(1);
        main = new QWidget();
        main->setObjectName(QString::fromUtf8("main"));
        main->setGeometry(QRect(0, 0, 839, 405));
        gridLayout_6 = new QGridLayout(main);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label_Q = new QLabel(main);
        label_Q->setObjectName(QString::fromUtf8("label_Q"));
        label_Q->setMinimumSize(QSize(120, 120));
        label_Q->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_Q);

        label_R = new QLabel(main);
        label_R->setObjectName(QString::fromUtf8("label_R"));
        label_R->setMinimumSize(QSize(120, 120));
        label_R->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_R);

        label_G = new QLabel(main);
        label_G->setObjectName(QString::fromUtf8("label_G"));
        label_G->setMinimumSize(QSize(120, 120));
        label_G->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_G);

        label_B = new QLabel(main);
        label_B->setObjectName(QString::fromUtf8("label_B"));
        label_B->setMinimumSize(QSize(120, 120));
        label_B->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_B);

        label_D = new QLabel(main);
        label_D->setObjectName(QString::fromUtf8("label_D"));
        label_D->setMinimumSize(QSize(120, 120));
        label_D->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_D);


        gridLayout_6->addLayout(horizontalLayout, 0, 0, 1, 1);

        toolBox->addItem(main, QString::fromUtf8("Main"));
        overview = new QWidget();
        overview->setObjectName(QString::fromUtf8("overview"));
        overview->setGeometry(QRect(0, 0, 839, 405));
        gridLayout_3 = new QGridLayout(overview);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        textBrowser = new QTextBrowser(overview);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));

        gridLayout_3->addWidget(textBrowser, 0, 0, 1, 1);

        toolBox->addItem(overview, QString::fromUtf8("1. Overview"));
        authors = new QWidget();
        authors->setObjectName(QString::fromUtf8("authors"));
        authors->setGeometry(QRect(0, 0, 839, 405));
        gridLayout_4 = new QGridLayout(authors);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        textBrowser_2 = new QTextBrowser(authors);
        textBrowser_2->setObjectName(QString::fromUtf8("textBrowser_2"));

        gridLayout_4->addWidget(textBrowser_2, 0, 0, 1, 1);

        toolBox->addItem(authors, QString::fromUtf8("2. Authors"));
        details = new QWidget();
        details->setObjectName(QString::fromUtf8("details"));
        details->setGeometry(QRect(0, 0, 839, 405));
        gridLayout_5 = new QGridLayout(details);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        textBrowser_3 = new QTextBrowser(details);
        textBrowser_3->setObjectName(QString::fromUtf8("textBrowser_3"));

        gridLayout_5->addWidget(textBrowser_3, 0, 0, 1, 1);

        toolBox->addItem(details, QString::fromUtf8("3. Details"));

        gridLayout_2->addWidget(toolBox, 0, 0, 1, 1);


        gridLayout->addWidget(frame, 0, 0, 1, 3);


        retranslateUi(QHelpDialog);

        toolBox->setCurrentIndex(1);
        toolBox->layout()->setSpacing(6);


        QMetaObject::connectSlotsByName(QHelpDialog);
    } // setupUi

    void retranslateUi(QDialog *QHelpDialog)
    {
        QHelpDialog->setWindowTitle(QCoreApplication::translate("QHelpDialog", "Dialog", nullptr));
        btn_ok->setText(QCoreApplication::translate("QHelpDialog", "OK", nullptr));
        label_Q->setText(QCoreApplication::translate("QHelpDialog", "TextLabel", nullptr));
        label_R->setText(QCoreApplication::translate("QHelpDialog", "TextLabel", nullptr));
        label_G->setText(QCoreApplication::translate("QHelpDialog", "TextLabel", nullptr));
        label_B->setText(QCoreApplication::translate("QHelpDialog", "TextLabel", nullptr));
        label_D->setText(QCoreApplication::translate("QHelpDialog", "TextLabel", nullptr));
        toolBox->setItemText(toolBox->indexOf(main), QCoreApplication::translate("QHelpDialog", "Main", nullptr));
        textBrowser->setHtml(QCoreApplication::translate("QHelpDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu Mono'; font-size:14pt; font-weight:700; font-style:italic;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:400;\">    </span><span style=\" color:#ef2929;\">Slam</span><span style=\" font-weight:400;\"> (simultaneous localization and mapping), also known as </span><span style=\" color:#ef2929;\">CML</span><span style=\" font-weight:400;\"> (concurrent mapping and localization), real-time positioning and map construction, or concurrent mapping and positioning. The problem can be described as: put a robot into an unknown position in an unknown environment, and whether there is a way for the robot to gradually draw"
                        " a complete map of the environment while moving. The so-called a consistent map refers to walking to every corner of the room without obstacles.</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:400;\">    </span><span style=\" color:#ef2929;\">V-SLAM</span><span style=\" font-weight:400;\"> is a branch of slam. It takes the camera as the data acquisition equipment to locate and build maps in real time through relevant image and estimation optimization algorithms. Relying on a special camera rgbd camera, this project realizes slam software based on RGB image and depth image based on </span>ORB-SLAM<span style=\" font-weight:400;\"> algorithm library, </span>Yolo<span style=\" font-weight:400;\"> image recognition library and </span>Open3D<span style=\" font-weight:400;\"> visualization library. It has the functions of location, scene recognition and scene reconstruction.</span></p>\n"
"<p style=\"-q"
                        "t-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\":/img/ros-server.png\" /></p></body></html>", nullptr));
        toolBox->setItemText(toolBox->indexOf(overview), QCoreApplication::translate("QHelpDialog", "1. Overview", nullptr));
        textBrowser_2->setHtml(QCoreApplication::translate("QHelpDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu Mono'; font-size:14pt; font-weight:700; font-style:italic;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#fb8b00;\">{</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">  <span style=\" color:#ef2929;\">&quot;developers&quot;</span>: <span style=\" color:#ad7fa8;\">[</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" color:#729fcf;\">{</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px;"
                        " -qt-block-indent:0; text-indent:0px;\">      <span style=\" color:#ef2929;\">&quot;name&quot;</span>: <span style=\" color:#4e9a06;\">&quot;something&quot;</span>,</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">      <span style=\" color:#ef2929;\">&quot;e-mail&quot;</span>: <span style=\" color:#4e9a06;\">&quot;something&quot;</span>,</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">      <span style=\" color:#ef2929;\">&quot;school&quot;</span>: <span style=\" color:#4e9a06;\">&quot;something&quot;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" color:#729fcf;\">}</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">  <span style=\" color:#ad7fa8;\">]</span></p>"
                        "\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#fb8b00;\">}</span></p></body></html>", nullptr));
        toolBox->setItemText(toolBox->indexOf(authors), QCoreApplication::translate("QHelpDialog", "2. Authors", nullptr));
        textBrowser_3->setHtml(QCoreApplication::translate("QHelpDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu Mono'; font-size:14pt; font-weight:700; font-style:italic;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">The workflow of the developed system is shown in the figure below. The solution method used this time is post-processing. The main process consists of two parts:</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">1) Data acquisition: the Kinect camera is used to collect color images, depth images and IMU data at a certain frame rate. This         part is based on a small comm"
                        "unication network composed of Ubuntu host, Android phone and Kinect camera. Through WiFi communication, Android phones send control signals to the Ubuntu host, receive data messages (including color images, depth images and IMU data) sent by the Ubuntu host, and display the data in real time on the user's Android selling price. The communication between Ubuntu and Kinect cameras is based on the ROS multi process communication mechanism. Specifically, Ubuntu will process the control messages sent by Android (such as start collection, stop collection, etc.) and convey them to the Kinect camera. The data collected by Kinect camera will be sent to Android via Ubuntu host for real-time display.</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">2) Data processing: that is, on Ubuntu, the qrgbd interface application developed based on QT is used to process and solve the collected data.</span></p"
                        ">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\":/img/qrgbd-system-small.png\" /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ef2929;\">    main interface thread</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">The main interface thread completes the configuration of data to be processed, interactive control of processing flow, display and output of processing results and other functions. Specifically, the main interface thread obtains the path of the corresponding data through the configuration dialog window and reads it into memory when processing begins. At the beginning of processing, the data frame is sent to the slam thread at a certain frame rate to complete the slam pro"
                        "cess of the current frame and obtain the current pose. Then the pose and data frame are sent to the reconstruction thread for scene reconstruction. At the same time, when the recognition thread is idle, the color image is sent to the recognition thread for object recognition, and the marked image is obtained and displayed.</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ef2929;\">    slam thread</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">Slam threads rely on the orb-slam3 library. When the main thread passes in the data frame, it performs the slam solution of the data frame. If the system has not been initialized, it will be initialized and then tracked. When the trace is lost, the slam system will try to relocate. If it fails, it will be reinitialized. After the related wo"
                        "rk is completed, the thread will return the calculated pose results to the main thread.</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ef2929;\">    identifying thread</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ef2929;\">    </span><span style=\" font-weight:400;\">Identifying threads relies on the yolo3 library.</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#ef2929;\">    rebuilding thread</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">The reconstruction thread completes the remapping of depth map and the reconstruction of 3D scene. Since 3D scene recons"
                        "truction takes a lot of time, it is put into the sub thread of the reconstruction thread:</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">1) Remapping of depth map: depth information of corresponding position pixels recorded in depth image acquired by rgbd camera. Because it is a single channel image, it is converted to a three channel RGB color image through remapping. After the remapping is completed, the resulting color image is returned to the main thread for visualization.</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">    <span style=\" font-weight:400;\">2) 3D scene reconstruction: it relies on PCL library and is completed in sub threads. When the reconstruction thread converts the pixels on the depth map to the world coordinate system with the help of pose information and depth information, "
                        "the point cloud data is obtained and sent to the 3D scene reconstruction sub thread. The 3D scene reconstruction sub thread visualizes the point cloud in an incremental manner. The 3D scene reconstruction sub thread has two visualization modes: interactive visualization and rendering visualization:</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">        <span style=\" font-weight:400;\">a) Interactive visualization: when the slam is completed or terminated, the interactive visualization mode is enabled. Users can manipulate the PCL point cloud window through the mouse to view the point cloud from multiple angles and levels;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">        <span style=\" font-weight:400;\">b) Rendering visualization: when slam is performed, the PCL window is performed in a rendering visualization manner. Specifical"
                        "ly, visualize the point cloud to be incrementally visualized in each frame, track the position of the camera, and change the viewport to simulate the perspective during data collection.</span></p></body></html>", nullptr));
        toolBox->setItemText(toolBox->indexOf(details), QCoreApplication::translate("QHelpDialog", "3. Details", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QHelpDialog: public Ui_QHelpDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QHELPDIALOG_H
