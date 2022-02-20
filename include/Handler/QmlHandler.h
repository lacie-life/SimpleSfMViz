#ifndef QMLHANDLER_H
#define QMLHANDLER_H

#include <QObject>
#include <QString>
#include <cstdarg>

#define QML_HANDLER QmlHandler::instance()

class QmlHandler : public QObject
{
    Q_OBJECT
    static QmlHandler* self;
public:
    static QmlHandler *instance();

public slots:
    void qmlSendEvent(int event);
    void qmlMessage(QString msg);

private:
    QmlHandler();
    QmlHandler(const QmlHandler& _other) = delete;
    void operator =(const QmlHandler& _other) = delete;

signals:
    void notifyQMLEvent(int event);
};

#endif // QMLHANDLER_H
