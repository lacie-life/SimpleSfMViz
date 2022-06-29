#include "configdialog.h"
#include "QCloseEvent"
#include "QFileDialog"
#include "QMessageBox"
#include "ui_configdialog.h"

ConfigDialog::ConfigDialog(QWidget *parent) : QDialog(parent),
                                              ui(new Ui::ConfigDialog) {
    ui->setupUi(this);
    this->setWindowTitle("Configure");
    this->connection();
    {
        // pre setting
        this->_temp_seqPath = this->_seqPath = "/home/csl/dataset/rgbd-slam/2022-6-2-8-19-15";
        this->_temp_colorImgPath = this->_colorImgPath = "/home/csl/dataset/rgbd-slam/2022-6-2-8-19-15/color";
        this->_temp_depthImgPath = this->_depthImgPath = "/home/csl/dataset/rgbd-slam/2022-6-2-8-19-15/depth";
        this->_temp_assoPath = this->_assoPath = "/home/csl/dataset/rgbd-slam/2022-6-2-8-19-15/info/associate.txt";
        // -----------
        this->_temp_settingPath = this->_settingPath = "/home/csl/TempWork/rgbd-slam/config/rgbd.yaml";
        this->_temp_vocPath = this->_vocPath = "/home/csl/SoftWare/ORB_SLAM3/Vocabulary/ORBvoc.txt";
        this->_temp_classes = this->_classes = "/home/csl/TempWork/rgbd-slam/yolov4-learn/yolo/coco.names";
        this->_temp_modelConfig = this->_modelConfig = "/home/csl/TempWork/rgbd-slam/yolov4-learn/yolo/yolov4.cfg";
        this->_temp_modelWeights = this->_modelWeights = "/home/csl/TempWork/rgbd-slam/yolov4-learn/yolo/yolov4.weights";
        // display
        this->display();
    }
}

ConfigDialog::~ConfigDialog() {
    delete ui;
}

void ConfigDialog::connection() {
    // sequence
    connect(ui->load_seq, &QPushButton::clicked, this, [=]() {
        QString seqPath;
        if (this->_temp_seqPath.isEmpty()) {
            seqPath = QFileDialog::getExistingDirectory(this, "Sequence Path", QDir::currentPath());
        } else {
            seqPath = QFileDialog::getExistingDirectory(this, "Sequence Path", this->_temp_seqPath);
        }

        if (seqPath.isEmpty()) {
            return;
        }
        this->_temp_seqPath = seqPath;

        this->_temp_colorImgPath = seqPath + "/color";
        this->_temp_depthImgPath = seqPath + "/depth";
        this->_temp_assoPath = seqPath + "/info/associate.txt";

        this->display();
    });
    // vocabulary
    connect(ui->load_voc, &QPushButton::clicked, this, [=]() {
        QString vocPath;
        if (this->_temp_vocPath.isEmpty()) {
            vocPath = QFileDialog::getOpenFileName(this, "Vocabulary Path", QDir::currentPath());
        } else {
            vocPath = QFileDialog::getOpenFileName(this, "Vocabulary Path", this->_temp_vocPath);
        }

        if (vocPath.isEmpty()) {
            return;
        }

        this->_temp_vocPath = vocPath;

        this->display();
    });
    // settings
    connect(ui->load_set, &QPushButton::clicked, this, [=]() {
        QString setPath;
        if (this->_temp_settingPath.isEmpty()) {
            setPath = QFileDialog::getOpenFileName(this, "Settings Path", QDir::currentPath());
        } else {
            setPath = QFileDialog::getOpenFileName(this, "Settings Path", this->_temp_settingPath);
        }

        if (setPath.isEmpty()) {
            return;
        }

        this->_temp_settingPath = setPath;

        this->display();
    });

    // classes
    connect(ui->load_classes, &QPushButton::clicked, this, [=]() {
        QString classPath;
        if (this->_temp_classes.isEmpty()) {
            classPath = QFileDialog::getOpenFileName(this, "Classes Path", QDir::currentPath());
        } else {
            classPath = QFileDialog::getOpenFileName(this, "Classes Path", this->_temp_classes);
        }

        if (classPath.isEmpty()) {
            return;
        }

        this->_temp_classes = classPath;

        this->display();
    });
    // modelConfig
    connect(ui->load_modelConfig, &QPushButton::clicked, this, [=]() {
        QString modelConfigPath;
        if (this->_temp_modelConfig.isEmpty()) {
            modelConfigPath = QFileDialog::getOpenFileName(this, "Model Config Path", QDir::currentPath());
        } else {
            modelConfigPath = QFileDialog::getOpenFileName(this, "Model Config Path", this->_temp_modelConfig);
        }

        if (modelConfigPath.isEmpty()) {
            return;
        }

        this->_temp_modelConfig = modelConfigPath;

        this->display();
    });
    // modelweight
    connect(ui->load_modelWeight, &QPushButton::clicked, this, [=]() {
        QString modelWeightPath;
        if (this->_temp_classes.isEmpty()) {
            modelWeightPath = QFileDialog::getOpenFileName(this, "Model Weights Path", QDir::currentPath());
        } else {
            modelWeightPath = QFileDialog::getOpenFileName(this, "Model Weights Path", this->_temp_modelWeights);
        }

        if (modelWeightPath.isEmpty()) {
            return;
        }

        this->_temp_modelWeights = modelWeightPath;

        this->display();
    });

    // cancel button
    connect(ui->btn_cancel, &QPushButton::clicked, this, [=]() {
        // restore
        this->_temp_seqPath = this->_seqPath;
        this->_temp_colorImgPath = this->_colorImgPath;
        this->_temp_depthImgPath = this->_depthImgPath;
        this->_temp_assoPath = this->_assoPath;

        this->_temp_settingPath = this->_settingPath;
        this->_temp_vocPath = this->_vocPath;

        this->close();
    });

    // ok button
    connect(ui->btn_ok, &QPushButton::clicked, this, [=]() {
        // should get path form lineedit
        this->_seqPath = this->_temp_seqPath = ui->lineEdit_seq->text();
        this->_assoPath = this->_temp_assoPath = ui->lineEdit_asso->text();
        this->_colorImgPath = this->_temp_colorImgPath = ui->lineEdit_color->text();
        this->_depthImgPath = this->_temp_depthImgPath = ui->lineEdit_depth->text();

        this->_settingPath = this->_temp_settingPath = ui->lineEdit_set->text();
        this->_vocPath = this->_temp_vocPath = ui->lineEdit_voc->text();

        this->close();
    });
}

void ConfigDialog::display() {
    // display the path accroding to the temp variables
    ui->lineEdit_seq->setText(this->_temp_seqPath);
    ui->lineEdit_color->setText(this->_temp_colorImgPath);
    ui->lineEdit_depth->setText(this->_temp_depthImgPath);
    ui->lineEdit_asso->setText(this->_temp_assoPath);

    ui->lineEdit_set->setText(this->_temp_settingPath);
    ui->lineEdit_voc->setText(this->_temp_vocPath);

    ui->lineEdit_class->setText(this->_temp_classes);
    ui->lineEdit_modelConfig->setText(this->_temp_modelConfig);
    ui->lineEdit_modelWeight->setText(this->_temp_modelWeights);
}

bool ConfigDialog::isSetted() {
    return !this->_seqPath.isEmpty() &
           !this->_assoPath.isEmpty() &
           !this->_colorImgPath.isEmpty() &
           !this->_depthImgPath.isEmpty() &
           !this->_settingPath.isEmpty() &
           !this->_vocPath.isEmpty() &
           !this->_classes.isEmpty() &
           !this->_modelConfig.isEmpty() &
           !this->_modelWeights.isEmpty();
}

void ConfigDialog::closeEvent(QCloseEvent *e) {
    // if the configure is changed, then emit a warning
    bool changed = (ui->lineEdit_seq->text() != this->_seqPath) ||
                   (ui->lineEdit_asso->text() != this->_assoPath) ||
                   (ui->lineEdit_color->text() != this->_colorImgPath) ||
                   (ui->lineEdit_depth->text() != this->_depthImgPath) ||
                   (ui->lineEdit_set->text() != this->_settingPath) ||
                   (ui->lineEdit_voc->text() != this->_vocPath) ||
                   (ui->lineEdit_class->text() != this->_classes) ||
                   (ui->lineEdit_modelConfig->text() != this->_modelConfig) ||
                   (ui->lineEdit_modelWeight->text() != this->_temp_modelWeights);
    if (changed) {
        auto choice = QMessageBox::information(
            this,
            "Attention",
            "Your configure has been changed, do you want to save it?",
            QMessageBox::StandardButton::Yes | QMessageBox::StandardButton::No);

        if (choice == QMessageBox::StandardButton::Yes) {
            ui->btn_ok->click();
        }
    }
    return QDialog::closeEvent(e);
}
