#ifndef QCAMERACAPTURE_H
#define QCAMERACAPTURE_H

#include <QObject>

class QcameraCapture : public QObject
{
    Q_OBJECT
public:
    explicit QcameraCapture(QObject *parent = nullptr);

signals:

};

#endif // QCAMERACAPTURE_H
