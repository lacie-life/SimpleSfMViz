#include "AppEngine.h"
#include "AppConstant.h"
#include "Screen_Def.h"
#include "AppEnums.h"
#include "AppModel.h"
#include "QmlHandler.h"
#include "QPointCloudViewer.h"
#include "Camera/QImageProvider.h"

#include <QVariant>

ScreenDef* ScreenDef::m_instance = nullptr;
QMutex ScreenDef::m_lock;
AppConstant* AppConstant::m_instance = nullptr;
QMutex AppConstant::m_lock;

AppEngine::AppEngine()
{
    m_rootContext = this->rootContext();
}

AppEngine::~AppEngine(){

}

void AppEngine::initEngine(){

    // register class
    qmlRegisterUncreatableType<AppEnums>("QmlCustomItem", 1, 0, "ENUMS", "Uncreatable");
    qmlRegisterUncreatableType<AppEnums>("AppEnums", 1, 0, "Enums", "Cannot create object from enums!");

    // connect signal slots
    connect(QML_HANDLER, &QmlHandler::notifyQMLEvent, this, &AppEngine::slotReceiveEvent);

    QImageProvider *liveImageProvider(new QImageProvider);

    // set context properties
    m_rootContext->setContextProperty("QmlConst", DEFS);
    m_rootContext->setContextProperty("QmlHandler", QML_HANDLER);
    m_rootContext->setContextProperty("QmlScreen", SCR_DEF);
    m_rootContext->setContextProperty("QmlModel", MODEL);

    m_rootContext->setContextProperty("comboboxModel", &MODEL->comboboxModel);
    m_rootContext->setContextProperty("progressDialog", &MODEL->m_progressDialog);

    m_rootContext->setContextProperty("liveImageProvider", liveImageProvider);
    this->addImageProvider("live", liveImageProvider);
    connect(MODEL, &AppModel::currentFrameChanged, liveImageProvider, &QImageProvider::updateImage);

}

void AppEngine::startEngine(){
    this->load(SCR_DEF->QML_APP());

//    this->pointCloudRenderScreenRun(this);
}

void AppEngine::pointCloudRenderScreenRun(AppEngine *engine)
{
    QPointCloudViewer *m_pointCloudRender = new QPointCloudViewer(engine);
    m_pointCloudRender->resize(600, 600);
    m_pointCloudRender->show();
}

void AppEngine::slotReceiveEvent(int event)
{
    CONSOLE << "Received event " << event;
    switch (event) {
    case static_cast<int>(AppEnums::EVT_NONE):
        CONSOLE << "Invalid event";
        break;
    case static_cast<int>(AppEnums::EVT_CLICK_HOME_SCREEN):
        CONSOLE << "Change screen";
        MODEL->setCurrentScreenID(AppEnums::HOME_SCREEN);
        break;
    case static_cast<int>(AppEnums::EVT_CLICK_PROCESS_SCREEN):
        CONSOLE << "Change screen";
        MODEL->setCurrentScreenID(AppEnums::PROCESS_SCREEN);
        break;
    case static_cast<int>(AppEnums::EVT_CLICK_CHOOSE_ROSBAG):
        CONSOLE << MODEL->rosBagPath();
        break;
    case static_cast<int>(AppEnums::EVT_CLICK_RUN):
        CONSOLE << "RUN ?";
        MODEL->cameraRun();
        break;
    default:
        break;
    }
    // bla bla bla
}
