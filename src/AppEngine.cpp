#include "AppEngine.h"
#include "AppConstant.h"
#include "Screen_Def.h"
#include "AppEnums.h"
#include "AppModel.h"
#include "QmlHandler.h"
#include "QPointCloudViewer.h"

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

    // set context properties
    m_rootContext->setContextProperty("QmlConst", DEFS);

    m_rootContext->setContextProperty("QmlHandler", QML_HANDLER);

    m_rootContext->setContextProperty("QmlScreen", SCR_DEF);

    m_rootContext->setContextProperty("QmlModel", MODEL);
}

void AppEngine::startEngine(){
    this->load(SCR_DEF->QML_APP());

    // this->pointCloudRenderScreenRun(this);
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
        // do sth here, maybe call a function to process images
        // then use MODEL->setCurrentPath to re-set path
        break;
    case static_cast<int>(AppEnums::EVT_CLICK_RUN_SfM):
        CONSOLE << "Run SfM Event";
        MODEL->runSfM("/home/lacie/Github/GreenHouseAR/assest/data/crazyhorse");
        break;
    default:
        break;
    }
    // bla bla bla
}
