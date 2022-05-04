#include "Camera/QImageProvider.h"
#include "AppConstant.h"

#include <QDebug>

QImageProvider::QImageProvider(QObject *parent)
    : QObject(parent),
      QQuickImageProvider(QQuickImageProvider::Image)
{
    m_image = QImage(200,200,QImage::Format_RGB32);
    m_image.fill(QColor("black"));
}

QImage QImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    Q_UNUSED(id);

    if(size){
        *size = m_image.size();
    }

    if(requestedSize.width() > 0 && requestedSize.height() > 0) {
        m_image = m_image.scaled(requestedSize.width(), requestedSize.height(), Qt::KeepAspectRatio);
    }

    return m_image;
}

void QImageProvider::updateImage(const QImage &image)
{
    if(!image.isNull()) {
        this->m_image = image;
        emit imageChanged();
    }
}
