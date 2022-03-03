#ifndef APPMODEL_H
#define APPMODEL_H

#include <QObject>

#include <QObject>
#include <QString>
#include <QMutex>
#include <QStringList>
#include <QVector>
#include <QTimer>
#include <QImage>
#include <QQmlApplicationEngine>

#include "AppEnums.h"
#include "SfM/QSfM.h"

#define MODEL AppModel::instance()

class AppModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int currentScreenID READ currentScreenID WRITE setCurrentScreenID NOTIFY currentScreenIDChanged)
    Q_PROPERTY(AppEnums::APP_STATE state READ state WRITE setState NOTIFY stateChanged)

public:

    static AppModel *instance();
    AppEnums::APP_STATE state() const;
    int currentScreenID() const;

    void runSfM(QString path);

public slots:
    void setState(AppEnums::APP_STATE state);
    void setCurrentScreenID(int currentScreenID);

signals:
    void stateChanged();
    void currentScreenIDChanged(int currentScreenID);

private:
    AppModel(QObject* parent = nullptr);
    AppModel(const AppModel& _other) = delete;
    void operator =(const AppModel& _other) = delete;

private:
    static AppModel* m_instance;
    static QMutex m_lock;

    QSfM *m_sfm;

    static AppEnums::APP_STATE m_state;
    int m_currentScreenID;
};

#endif // APPMODEL_H
