#ifndef APPMODEL_H
#define APPMODEL_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QStringList>
#include <QVector>
#include <QTimer>
#include <QImage>
#include <QQmlApplicationEngine>
#include <QStringListModel>

#include <opencv2/opencv.hpp>

#include "AppEnums.h"
#include "QConfig.h"
#include "SfM/QSfM.h"
#include "DataVisualize/QProgressBarDialog.h"
#include "Camera/QCameraCapture.h"

#define MODEL AppModel::instance()

// TODO: Still test with video path with variable rosBag

class AppModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int currentScreenID READ currentScreenID WRITE setCurrentScreenID NOTIFY currentScreenIDChanged)
    Q_PROPERTY(AppEnums::APP_STATE state READ state WRITE setState NOTIFY stateChanged)
    Q_PROPERTY(QString rosBagPath READ rosBagPath WRITE setRosBagPath NOTIFY rosBagPathChanged)
    Q_PROPERTY(int detectModel READ detectModel WRITE setDetectModel NOTIFY detectModelChanged)

public:

    static AppModel *instance();
    AppEnums::APP_STATE state() const;
    int currentScreenID() const;
    QString rosBagPath() const;
    int detectModel() const;

    void setDefaultConfig();

    void runSfM(QString path);

public slots:
    void setState(AppEnums::APP_STATE state);
    void setCurrentScreenID(int currentScreenID);
    void setRosBagPath(QString path);
    void setDetectModel(int model);
    void setCurrentFrame(cv::Mat *frame);

signals:
    void stateChanged();
    void currentScreenIDChanged(int currentScreenID);
    void rosBagPathChanged(QString path);
    void detectModelChanged(int model);
    void currentFrameChanged(QImage &image);

private:
    AppModel(QObject* parent = nullptr);
    AppModel(const AppModel& _other) = delete;
    void operator =(const AppModel& _other) = delete;

private:
    static AppModel* m_instance;
    static QMutex m_lock;

    static AppEnums::APP_STATE m_state;
    int m_currentScreenID;

    QString m_rosBag;
    QConfig* m_config;
    QCameraCapture* m_camera;
    QImage m_currentFrame;

public:
    QProgressBarDialog m_progressDialog;
    QStringListModel comboboxModel;
};

#endif // APPMODEL_H
