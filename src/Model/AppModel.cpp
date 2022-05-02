#include "AppModel.h"
#include "AppConstant.h"
#include <QDebug>
#include <QString>
#include <QThread>

AppModel* AppModel::m_instance = nullptr;
QMutex AppModel::m_lock;
AppEnums::APP_STATE AppModel::m_state = AppEnums::APP_STATE::STATE_NONE;

AppModel::AppModel(QObject *parent)
    : QObject{parent}
    , m_currentScreenID{static_cast<int>(AppEnums::HOME_SCREEN)}
{
    CONSOLE << "Init instance";

    setDefaultConfig();

    connect(m_camera, &QCameraCapture::frameCaptured, this, &AppModel::setCurrentFrame);
}

AppModel *AppModel::instance(){
    m_lock.lock();
    if (nullptr == m_instance){
        m_instance = new AppModel();
    }
    m_lock.unlock();
    return m_instance;
}

AppEnums::APP_STATE AppModel::state() const
{
    return m_state;
}

int AppModel::currentScreenID() const
{
    return m_currentScreenID;
}

QString AppModel::rosBagPath() const
{
    return m_rosBag;
}

int AppModel::detectModel() const
{
    return m_config->modelType();
}

void AppModel::setDefaultConfig()
{
    // Combobox Model
    QStringList modelList;
    for(int i = 0; i < AppEnums::MODEL_ZOO.size() - 1; i++)
    {
        modelList << AppEnums::MODEL_ZOO[i];
    }
    comboboxModel.setStringList(modelList);

    // QCameraCapture
    m_camera = new QCameraCapture(nullptr);

    //QConfig
    m_config = new QConfig(nullptr);
}

void AppModel::runSfM(QString path)
{
    QSfM *sfm = new QSfM(this);

    sfm->init(path);
    sfm->run();
}

void AppModel::setState(AppEnums::APP_STATE state)
{
    m_state = state;

    emit stateChanged();
}

void AppModel::setCurrentScreenID(int currentScreenID)
{
    if(m_currentScreenID == currentScreenID){
        return;
    }

    m_currentScreenID = currentScreenID;
    emit currentScreenIDChanged(m_currentScreenID);
}

void AppModel::setRosBagPath(QString path)
{
    m_rosBag = path;

    CONSOLE << m_rosBag;

    emit rosBagPathChanged(m_rosBag);
}

void AppModel::setDetectModel(int model)
{
    m_config->setModelType((AppEnums::DETECT_MODEL)model);

    CONSOLE << "Model: " << AppEnums::MODEL_ZOO[model];

    emit detectModelChanged(m_config->modelType());
}

void AppModel::setCurrentFrame(Mat *frame)
{
    QImage img = QImage(frame->data,frame->cols,frame->rows,QImage::Format_RGB888).rgbSwapped();
    m_currentFrame = img;

    emit currentFrameChanged(m_currentFrame);
}



