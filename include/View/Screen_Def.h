#ifndef SCREENDEF_H
#define SCREENDEF_H

#include <QObject>
#include <QString>
#include <QMutex>
#include <QDebug>
#include <QCoreApplication>
#include <QHash>
#include "AppEnums.h"

#ifndef SCREEN_DEF_MACROS
#define SCREEN_DEF_MACROS

#define DEF_SCREEN(screenName, link) Q_PROPERTY(QString screenName READ screenName CONSTANT) \
    public: QString screenName() const { return QString(link); }

#endif

#ifndef BUILD_DIR
#define BUILD_DIR QCoreApplication::applicationDirPath()

#endif

#define SCR_DEF ScreenDef::getInstance()
#define DELETE_SCR_DEF ScreenDef::DestroyInstance()

class ScreenDef : public QObject
{
    Q_OBJECT
    static ScreenDef* m_instance;
    static QMutex m_lock;
public:
    static ScreenDef* getInstance(){
        m_lock.lock();
        if (nullptr == m_instance){
            m_instance = new ScreenDef;
        }
        m_lock.unlock();
        return m_instance;
    }

    static void DestroyInstance(){
        m_lock.lock();
        if(nullptr != m_instance){
            delete m_instance;
        }
        m_instance = nullptr;
        m_lock.unlock();
    }

private:
    ScreenDef(QObject* parent = nullptr) : QObject( parent ) {}
    ~ScreenDef() {}
    ScreenDef(const ScreenDef&) = delete;
    void operator =(const ScreenDef&) = delete;

    DEF_SCREEN(QML_FOLDER       , "file:/home/jun/Github/GreenHouseAR/assest/qml/")
    DEF_SCREEN(QML_APP     , QML_FOLDER() + "main.qml")

signals:

};

#endif // SCREENDEF_H
