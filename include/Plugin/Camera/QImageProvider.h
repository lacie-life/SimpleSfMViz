#ifndef QIMAGEPROVIDER_H
#define QIMAGEPROVIDER_H

#include <QObject>
#include <QQuickImageProvider>
#include <QCache>

class QImageProvider : public QObject, public QQuickImageProvider
{
    Q_OBJECT
public:
    explicit QImageProvider(QObject *parent = nullptr);

    QPixmap requestPixmap(const QString& id, QSize* size, const QSize& requestedSize) override;

public slots:
    void updateImage(const QImage &image);

signals:
    void imageChanged();

private:
    QPixmap m_image;

};

#endif // QIMAGEPROVIDER_H
