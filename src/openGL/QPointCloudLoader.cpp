#include "openGL/QPointCloudLoader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <QDebug>

QPointCloudLoader::QPointCloudLoader(QObject *parent)
    : QObject{parent}
    , m_pointCloud(new QPointCloud())
{

}

QPointCloudLoader::QPointCloudLoader(const QString &filePath)
{
    setFilename(filePath);
}

QString QPointCloudLoader::filename() const
{
    return m_filename;
}

QPointCloud *QPointCloudLoader::pointCloud() const
{
    return m_pointCloud;
}

void QPointCloudLoader::setFilename(QString filename)
{
    if (m_filename == filename)
        return;

    if(filename.endsWith(".pcd", Qt::CaseInsensitive))
    {
        pcl::PCDReader reader;
        reader.read(filename.toStdString(), *m_pointCloud->pointCloud());


    }
    else if(filename.endsWith(".ply", Qt::CaseInsensitive))
    {
        pcl::PLYReader reader;
        reader.read(filename.toStdString(), *m_pointCloud->pointCloud());
    }
    qDebug() << "Read Pointcloud" << filename << "with" << ((m_pointCloud->pointCloud()->width) * (m_pointCloud->pointCloud()->height)) << "points.";
    m_filename = filename;
    emit filenameChanged(filename);
    emit pointCloudChanged(m_pointCloud);
}
