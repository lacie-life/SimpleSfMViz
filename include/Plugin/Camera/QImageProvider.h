#ifndef QIMAGEPROVIDER_H
#define QIMAGEPROVIDER_H

#include <QObject>

class QImageProvider : public QObject
{
    Q_OBJECT
public:
    explicit QImageProvider(QObject *parent = nullptr);

signals:

};

#endif // QIMAGEPROVIDER_H
