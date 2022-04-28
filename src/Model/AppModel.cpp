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

    emit rosBagPathChanged(m_rosBag);
}



