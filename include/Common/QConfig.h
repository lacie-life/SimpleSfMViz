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

class QConfig : public QObject
{
    Q_OBJECT
public:
    explicit QConfig(QObject *parent = nullptr);

signals:

};

#endif // QCONFIG_H
