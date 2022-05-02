#include "Camera/QImageProvider.h"
#include "AppConstant.h"

#include <QDebug>

QImageProvider::QImageProvider(QObject *parent)
    : QQuickImageProvider(QQuickImageProvider::Pixmap)
{

}

QPixmap QImageProvider::requestPixmap(const QString &id, QSize *size, const QSize &requestedSize)
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
        // CONSOLE << "Bug";
        this->m_image = QPixmap::fromImage(image);
        emit imageChanged();
    }
}
