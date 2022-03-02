#ifndef QIMAGELOADER_H
#define QIMAGELOADER_H

#include <QObject>

class QImageLoader : public QObject
{
    Q_OBJECT
public:
    explicit QImageLoader(QObject *parent = nullptr);

signals:

};

#endif // QIMAGELOADER_H
