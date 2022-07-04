#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QCSVHelper.h"
#include "QEmbeddedWindow.h"

// Ref: https://stackoverflow.com/questions/25311908/build-error-qt-headers-before-x11
#include <X11/Xlib.h>
#undef None
#undef Unsorted
#include <QCloseEvent>
#include <QFileDialog>
#include <QMessageBox>
#include <QDir>
#include "pcl/io/pcd_io.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("RGB-D Slam Handler");
    // create variables
    this->init();
    // build connections
    this->connection();
    // create opencv windows
    this->createCVWins();
}

MainWindow::~MainWindow() {
    // delete operations
    delete this->m_slam;
    delete this->m_rebuilder;
    delete this->m_recognizer;
    delete ui;
}

void MainWindow::connection() {
    // for start button
    connect(ui->btn_start, &QPushButton::clicked,
            this, [=]() {
                ui->stackedWidget->setCurrentWidget(ui->page_use);
            });
    // for help button 2
    connect(ui->btn_help_2, &QPushButton::clicked,
            this, [=]() {
                ui->btn_help->click();
            });
    // for back button
    connect(ui->btn_back, &QPushButton::clicked,
            this, [=]() {
                ui->stackedWidget->setCurrentWidget(ui->page_main);
            });
    // for save button
    connect(ui->btn_save, &QPushButton::clicked,
            this, [=]() {
                if (ui->tabW_mappoint->rowCount() == 0 && ui->tabW_keyframes->rowCount() == 0) {
                    // no data
                    QMessageBox::information(this, "Attention", "No data to save!");
                    return;
                }
                auto path = QFileDialog::getExistingDirectory(this, "save data path", QDir::currentPath());
                if (path.isEmpty()) {
                    return;
                }

                auto slamSystem = this->m_slam->getSlamSystem();
                const auto atlas = slamSystem->mpAtlas;
                std::vector<ORB_SLAM3::MapPoint *> mapPoints = atlas->GetAllMapPoints();
                std::vector<ORB_SLAM3::KeyFrame *> keyFrames = atlas->GetAllKeyFrames();

                // save map points and key frames info
                auto mapPointsFile = ns_csv::CSVWriter::create(path.toStdString() + "/mapPoints.csv");
                auto keyFramesFile = ns_csv::CSVWriter::create(path.toStdString() + "/keyFrames.csv");
                mapPointsFile->setPrecision(6);
                keyFramesFile->setPrecision(6);

                mapPointsFile->writeLine(',', "id", "X(M)", "Y(M)", "Z(M)");

                for (auto mpPtr : mapPoints) {
                    if (mpPtr != nullptr) {
                        // write data
                        auto pos = mpPtr->GetWorldPos();
                        mapPointsFile->writeLine(',', mpPtr->mnId, pos(0), pos(1), pos(2));
                    }
                }
                keyFramesFile->writeLine(',', "id", "Qx", "Qy", "Qz", "Qw", "X(M)", "Y(M)", "Z(M)");

                for (auto framePtr : keyFrames) {
                    if (framePtr != nullptr) {
                        // write data
                        auto rot = Eigen::Quaternionf(framePtr->GetRotation()).normalized();
                        auto trans = framePtr->GetTranslation();

                        keyFramesFile->writeLine(',', framePtr->mnId, rot.x(), rot.y(), rot.z(), rot.w(),
                                                 trans(0), trans(1), trans(2));
                    }
                }

                auto cloud = this->m_rebuilder->m_pcl_visual->m_allPts;

                // save pcd file
                pcl::io::savePCDFile(path.toStdString() + "/scene.pcd", *cloud, true);

                // finished
                QMessageBox::information(this, "Attention", "Save finished!");
            });
    // for quit button
    connect(ui->btn_quit, &QPushButton::clicked,
            this, [=]() {
                this->close();
            });
    // for help button
    connect(ui->btn_help, &QPushButton::clicked,
            this, [=]() {
                this->m_helpDig.exec();
            });
    // for configure button
    connect(ui->btn_config, &QPushButton::clicked,
            this, [=]() {
                this->m_configDig.display();
                this->m_configDig.exec();
            });
    // init _rebuilder
    connect(this, &MainWindow::signalInitRebuilder,
            this->m_rebuilder, &QRebuilder::init);
    // init _recognizer
    connect(this, &MainWindow::signalInitRecoginzer,
            this->m_recognizer, &QRecognizer::init);
    // [2] create slam system
    connect(this, &MainWindow::signalCreateSlamSystem,
            this->m_slam, &QSlam::createSlamSystem);
    // [3] run slam system
    connect(this->m_slam, &QSlam::createSlamSystemFinished,
            this, [=](float scale) {
                // show message
                this->statusBar()->showMessage("Create slam system finished, image scale is " +
                                               QString::number(scale, 'f', 3) + ", " +
                                               QString::fromStdString(this->m_timing.last_elapsed("cost")));

                // get image scale, set current frame index to zero
                this->m_imgScale = scale;
                this->m_curFrameIdx = -1;

                // start slaming [process a new frame]
                this->m_timer.singleShot(0, this, &MainWindow::processNewFrame);
            });
    // [1] run button [read config files, create slam system]
    connect(ui->btn_run, &QPushButton::clicked,
            this, [=]() {
                // Check whether the configuration is complete
                if (!this->m_configDig.isSetted()) {
                    QMessageBox::information(
                        this,
                        "Info",
                        "Your settings are incomplete. Please set the relevant path first.");
                    return;
                }

                // load configure info from files
                this->loadImages(this->m_configDig.m_assoPath.toStdString());

                // get total image size
                this->m_nImages = this->m_vstrImageFilenamesRGB.size();
                this->m_vTimesTrack.resize(m_nImages);

                // if no images or quantity does not correspond, don't process
                if (this->m_vstrImageFilenamesRGB.empty()) {
                    QMessageBox::information(
                        this,
                        "Info",
                        "No images found in provided path.");
                    return;
                } else if (this->m_vstrImageFilenamesD.size() != this->m_vstrImageFilenamesRGB.size()) {
                    QMessageBox::information(
                        this,
                        "Info",
                        "Different number of images for rgb and depth.");
                    return;
                }

                // start the threads
                this->m_slamThread.start();
                this->m_rebulidThread.start();
                this->m_recogThread.start();

                // start timing
                this->m_timing.reStart();

                // some settings
                this->m_isRunning = true;

                // some settings for buttons
                ui->btn_run->setEnabled(false);
                ui->btn_config->setEnabled(false);
                ui->btn_continue->setEnabled(false);

                // init
                emit this->signalInitRebuilder(&this->m_configDig);
                emit this->signalInitRecoginzer(&this->m_configDig);
                emit this->signalCreateSlamSystem(&this->m_configDig);

                // sho message
                this->statusBar()->showMessage("creating a new slam system.");
            });
    // [5] finish the frame
    connect(this->m_slam, &QSlam::processNewFrameFinished,
            this, [=](Sophus::SE3f pose) {
                // send to the rebulider
                emit this->signalNewDepthFrameToReBulider(this->m_imRGB, this->m_imD, pose);

                this->statusBar()->showMessage("Process frame [" + QString::number(this->m_curFrameIdx) + "] finished.");

                // organize a new frame
                double ttrack = this->m_timing.last_elapsed<ns_timer::DurationType::S>();
                m_vTimesTrack[this->m_curFrameIdx] = ttrack;

                // compute gather cost time
                double T = 0;
                if (this->m_curFrameIdx < m_nImages - 1) {
                    T = m_vTimestamps[this->m_curFrameIdx + 1] - m_tframe;
                } else if (this->m_curFrameIdx > 0) {
                    T = m_tframe - m_vTimestamps[this->m_curFrameIdx - 1];
                }

                qDebug() << "'frame idx':" << this->m_curFrameIdx
                         << ", 'gather delay':" << QString::number(T, 'f', 5)
                         << ", 'track delay':" << QString::number(ttrack, 'f', 5)
                         << ", 'tframe':" << QString::number(m_tframe, 'f', 5);

                // next frame
                if (ttrack < T)
                    this->m_timer.singleShot(T - ttrack, this, &MainWindow::processNewFrame);
                else
                    this->m_timer.singleShot(0, this, &MainWindow::processNewFrame);
            });
    // [4] send a new frame
    connect(this, &MainWindow::signalNewFrameToSlamSystem,
            this->m_slam, &QSlam::processNewFrame);
    // stop
    connect(ui->btn_stop, &QPushButton::clicked,
            this, [=]() {
                if (!this->m_isRunning) {
                    return;
                }
                this->m_isRunning = false;
                ui->btn_continue->setEnabled(true);
                ui->btn_stop->setEnabled(false);
                this->m_rebuilder->changeToInterMode();

                this->statusBar()->showMessage("process stoped.");
            });
    // continue
    connect(ui->btn_continue, &QPushButton::clicked,
            this, [=]() {
                if (this->m_curFrameIdx > 0 && this->m_isRunning == false) {
                    this->m_rebuilder->changeToRenderMode();
                    this->m_isRunning = true;
                    ui->btn_stop->setEnabled(true);
                    ui->btn_continue->setEnabled(false);
                    this->m_timer.singleShot(0, this, &MainWindow::processNewFrame);
                }
            });
    // force quit
    connect(ui->btn_forcequit, &QPushButton::clicked,
            this, [=]() {
                if (this->m_curFrameIdx < 0) {
                    return;
                }
                this->m_curFrameIdx = -1;
                this->m_isRunning = false;

                ui->btn_run->setEnabled(true);
                ui->btn_config->setEnabled(true);

                this->quitThreads();

                this->statusBar()->showMessage("quit all threads");

                this->displayMapInfo();

                this->m_slam->shutdown();

                this->m_rebuilder->changeToInterMode();
            });
    // new color frame
    connect(this, &MainWindow::signalNewColorFrameToRecongnizer,
            this->m_recognizer, &QRecognizer::processNewColorFrame);
    // recognize finished
    connect(this->m_recognizer, &QRecognizer::signalProcessNewFrameFinished,
            this, [=](cv::Mat img) {
                cv::imshow(this->m_recognizer->m_cvWinName.c_str(), img);
                if (this->m_isRunning) {
                    emit this->signalNewColorFrameToRecongnizer(m_imRGB);
                }
            });
    // rebuild finished
    connect(this->m_rebuilder, &QRebuilder::signalProcessNewFrameFinished,
            this, [=](cv::Mat img) {
                cv::imshow(this->m_rebuilder->m_cvWinName.c_str(), img);
            });
    // new depth frame
    connect(this, &MainWindow::signalNewDepthFrameToReBulider,
            this->m_rebuilder, &QRebuilder::processNewDepthFrame);
}

void MainWindow::init() {
    // set "QRGBD" images
    ui->label_Q_2->setPixmap(QPixmap(":/images/data/imgs/q.png").scaledToWidth(ui->label_Q_2->width()));
    ui->label_R_2->setPixmap(QPixmap(":/images/data/imgs/r.png").scaledToWidth(ui->label_R_2->width()));
    ui->label_G_2->setPixmap(QPixmap(":/images/data/imgs/g.png").scaledToWidth(ui->label_G_2->width()));
    ui->label_B_2->setPixmap(QPixmap(":/images/data/imgs/b.png").scaledToWidth(ui->label_B_2->width()));
    ui->label_D_2->setPixmap(QPixmap(":/images/data/imgs/d.png").scaledToWidth(ui->label_D_2->width()));

    qDebug() << "main thread: " << QThread::currentThread();

    this->statusBar()->setFont(QFont("Ubuntu Mono", -1, -1, true));

    this->m_slam = new QSlam();
    this->m_slam->moveToThread(&this->m_slamThread);

    this->m_recognizer = new QRecognizer();
    this->m_recognizer->moveToThread(&this->m_recogThread);

    this->m_rebuilder = new QRebuilder();
    this->m_rebuilder->moveToThread(&this->m_rebulidThread);
}

void MainWindow::closeEvent(QCloseEvent *e) {
    // quit running thread
    this->quitThreads();
    return QMainWindow::closeEvent(e);
}

void MainWindow::loadImages(const std::string &strAssociationFilename) {
    // load [imageName, association] from folder
    this->m_vstrImageFilenamesD.clear();
    this->m_vstrImageFilenamesRGB.clear();
    this->m_vTimestamps.clear();

    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while (!fAssociation.eof()) {
        std::string s;
        getline(fAssociation, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB, sD;
            ss >> t;
            m_vTimestamps.push_back(t);
            ss >> sRGB;
            m_vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            m_vstrImageFilenamesD.push_back(sD);
        }
    }
}

void MainWindow::processNewFrame() {
    if (!this->m_isRunning) {
        return;
    }

    ++this->m_curFrameIdx;

    // if all images have been processed, then return
    if (this->m_curFrameIdx == this->m_vstrImageFilenamesRGB.size()) {
        this->statusBar()->showMessage("process for all frames finished.");
        qDebug() << "slam finished!";
        ui->btn_forcequit->click();
        return;
    }

    // load color and depth images
    this->m_imD = cv::imread(this->m_configDig.m_seqPath.toStdString() + "/" +
                                m_vstrImageFilenamesD[this->m_curFrameIdx],
                            cv::IMREAD_UNCHANGED);
    this->m_imRGB = cv::imread(this->m_configDig.m_seqPath.toStdString() + "/" +
                                  m_vstrImageFilenamesRGB[this->m_curFrameIdx],
                              cv::IMREAD_UNCHANGED);

    // get frame time
    m_tframe = this->m_vTimestamps[this->m_curFrameIdx];

    // check color image
    if (this->m_imRGB.empty()) {
        QMessageBox::information(
            this,
            "Info",
            "Failed to load image at: " + this->m_configDig.m_seqPath +
                "/" + QString::fromStdString(this->m_vstrImageFilenamesRGB[this->m_curFrameIdx]));
        this->quitThreads();

        ui->btn_run->setEnabled(true);
        ui->btn_config->setEnabled(true);

        return;
    }

    // resize the image
    if (this->m_imgScale != 1.f) {
        int width = m_imRGB.cols * m_imgScale;
        int height = m_imRGB.rows * m_imgScale;
        cv::resize(m_imRGB, m_imRGB, cv::Size(width, height));
        cv::resize(m_imD, m_imD, cv::Size(width, height));
    }

    // restart timing
    this->m_timing.reStart();

    // process a new frame

    // slam
    emit this->signalNewFrameToSlamSystem(m_imRGB, m_imD, m_tframe);

    if (this->m_curFrameIdx == 0) {
        emit this->signalNewColorFrameToRecongnizer(m_imRGB);
    }
}

void MainWindow::quitThreads() {
    // quit the running thread
    if (this->m_slamThread.isRunning()) {
        this->m_slamThread.quit();
        this->m_slamThread.wait();
    }

    if (this->m_recogThread.isRunning()) {
        this->m_recogThread.quit();
        this->m_recogThread.wait();
    }

    //    if (this->m_rebulidThread.isRunning()) {
    //        this->m_rebulidThread.quit();
    //        this->m_rebulidThread.wait();
    //    }
}

void MainWindow::displayMapInfo() {
    this->displayMapPoints();
    this->displayKeyFrames();
    ui->tabWidget->setCurrentIndex(1);
}

void MainWindow::displayMapPoints() {
    auto slamSystem = this->m_slam->getSlamSystem();
    const auto atlas = slamSystem->mpAtlas;
    std::vector<ORB_SLAM3::MapPoint *> mapPoints = atlas->GetAllMapPoints();
    int num = 0;
    for (auto mpPtr : mapPoints) {
        if (mpPtr != nullptr) {
            ++num;
        }
    }
    ui->tabW_mappoint->setColumnCount(4);
    ui->tabW_mappoint->setHorizontalHeaderLabels({"id", "X(M)", "Y(M)", "Z(M)"});
    ui->tabW_mappoint->setRowCount(num);
    QTableWidgetItem *item;
    int idx = 0;
    for (auto mpPtr : mapPoints) {
        if (mpPtr != nullptr) {
            item = new QTableWidgetItem(QString::number(mpPtr->mnId));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 0, item);

            auto pos = mpPtr->GetWorldPos();

            item = new QTableWidgetItem(QString::number(pos(0), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 1, item);

            item = new QTableWidgetItem(QString::number(pos(1), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 2, item);

            item = new QTableWidgetItem(QString::number(pos(2), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_mappoint->setItem(idx, 3, item);

            ++idx;
        }
    }
}

void MainWindow::displayKeyFrames() {
    auto slamSystem = this->m_slam->getSlamSystem();
    const auto &atlas = slamSystem->mpAtlas;
    std::vector<ORB_SLAM3::KeyFrame *> keyFrames = atlas->GetAllKeyFrames();
    int num = 0;
    for (auto framePtr : keyFrames) {
        if (framePtr != nullptr) {
            ++num;
        }
    }

    int idx = 0;
    ui->tabW_keyframes->setColumnCount(8);
    ui->tabW_keyframes->setHorizontalHeaderLabels({"id", "Qx", "Qy", "Qz", "Qw", "X(M)", "Y(M)", "Z(M)"});
    ui->tabW_keyframes->setRowCount(num);
    QTableWidgetItem *item;
    for (auto framePtr : keyFrames) {
        if (framePtr != nullptr) {
            auto rot = Eigen::Quaternionf(framePtr->GetRotation()).normalized();
            auto trans = framePtr->GetTranslation();

            item = new QTableWidgetItem(QString::number(framePtr->mnId));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 0, item);

            item = new QTableWidgetItem(QString::number(rot.x(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 1, item);

            item = new QTableWidgetItem(QString::number(rot.y(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 2, item);

            item = new QTableWidgetItem(QString::number(rot.z(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 3, item);

            item = new QTableWidgetItem(QString::number(rot.w(), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 4, item);

            item = new QTableWidgetItem(QString::number(trans(0), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 5, item);

            item = new QTableWidgetItem(QString::number(trans(1), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 6, item);

            item = new QTableWidgetItem(QString::number(trans(2), 'f', 3));
            item->setTextAlignment(Qt::AlignCenter);
            ui->tabW_keyframes->setItem(idx, 7, item);

            ++idx;
        }
    }
}

void MainWindow::createCVWins() {
    // create the first window
    auto cvWin = cvEmbedWindow(this->m_rebuilder->m_cvWinName);
    ui->layout_rebuilder->addWidget(cvWin);
    auto img = cv::imread("./data/imgs/depth.png", cv::IMREAD_UNCHANGED);
    cv::imshow(this->m_rebuilder->m_cvWinName.c_str(), img);

    // create the second window
    QTimer::singleShot(0, this, [=]() {
        auto cvWin = cvEmbedWindow(this->m_recognizer->m_cvWinName);
        ui->layout_recognizer->addWidget(cvWin);
        auto img = cv::imread("./data/imgs/recognizer.png", cv::IMREAD_UNCHANGED);
        cv::imshow(this->m_recognizer->m_cvWinName.c_str(), img);
    });
}

void MainWindow::on_actionrun_triggered() {
    ui->btn_run->click();
}

void MainWindow::on_actionstop_triggered() {
    ui->btn_stop->click();
}

void MainWindow::on_actioncontinue_triggered() {
    ui->btn_continue->click();
}

void MainWindow::on_actionforce_quit_triggered() {
    ui->btn_forcequit->click();
}

void MainWindow::on_actionhelp_triggered() {
    ui->btn_help->click();
}

void MainWindow::on_actionback_triggered() {
    ui->btn_back->click();
}

void MainWindow::on_actionsave_triggered() {
    ui->btn_save->click();
}

void MainWindow::on_actionquit_triggered() {
    ui->btn_quit->click();
}

void MainWindow::on_actionstart_triggered() {
    ui->btn_start->click();
}


