#ifndef QCONFIG_H
#define QCONFIG_H

#include <QObject>
#include <QDebug>
#include <QCoreApplication>
#include <QFile>
#include <QDir>
#include <QVector>

#include <opencv2/core/core.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <assert.h>
#include <memory>

#include "AppEnums.h"

class QConfig : public QObject
{
    Q_OBJECT
    Q_PROPERTY(AppEnums::DETECT_MODEL modelType READ modelType WRITE setModelType NOTIFY modelTypeChanged)

public:
    explicit QConfig(QObject *parent = nullptr);

    AppEnums::DETECT_MODEL modelType() const;

public slots:
    void setModelType(AppEnums::DETECT_MODEL model);

signals:
    void modelTypeChanged(AppEnums::DETECT_MODEL model);

private:
    AppEnums::DETECT_MODEL currentModel;

};

#endif // QCONFIG_H
