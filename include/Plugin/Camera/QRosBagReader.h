#ifndef QROSBAGREADER_H
#define QROSBAGREADER_H

#include <QObject>

class QRosBagReader : public QObject
{
    Q_OBJECT
public:
    explicit QRosBagReader(QObject *parent = nullptr);

signals:

};

#endif // QROSBAGREADER_H
