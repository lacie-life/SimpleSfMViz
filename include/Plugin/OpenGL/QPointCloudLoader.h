#ifndef QPOINTCLOUDLOADER_H
#define QPOINTCLOUDLOADER_H

#include <QObject>
#include <QString>

class QPointCloud;

class QPointCloudLoader : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString filename READ filename WRITE setFilename NOTIFY filenameChanged)
    Q_PROPERTY(QPointCloud *pointCloud READ pointCloud NOTIFY pointCloudChanged)
public:
    QPointCloudLoader(QObject *parent = nullptr);
    QPointCloudLoader(const QString& filePath);

    QString filename() const;

    QPointCloud *pointCloud() const;

public slots:
    void setFilename(QString filename);

signals:
    void filenameChanged(QString filename);
    void pointCloudChanged(QPointCloud * pointcloud);

private:
    QString m_filename;
    QPointCloud *m_pointCloud;
};

#endif // QPOINTCLOUDLOADER_H
