#include "QmlHandler.h"
#include "AppConstant.h"
#include <QDebug>

QmlHandler *QmlHandler::self = nullptr;

QmlHandler *QmlHandler::instance()
{
    if (nullptr == self)
    {
        self = new QmlHandler();
    }
    return self;
}

void QmlHandler::qmlSendEvent(int event)
{
    CONSOLE << "Event " << event << " sent";
    emit notifyQMLEvent(event);
}

void QmlHandler::qmlMessage(QString msg)
{
    CONSOLE << msg;
}

QmlHandler::QmlHandler()
{
    CONSOLE << "Init instance";
}
