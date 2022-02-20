#ifndef QPOINTCLOUD_H
#define QPOINTCLOUD_H

#include <QObject>
#include <QVector3D>
#include <QQmlListProperty>
#include <QVector2D>
#include <QVector4D>

#include "QPointField.h"
#include "QPointCloudLoader.h"

namespace pcl {
    class PCLPointCloud2;
}

class QPointCloudPrivate;

class QPointCloud : public QObject
{
    Q_OBJECT
    Q_PROPERTY(quint32 height READ height WRITE setHeight NOTIFY heightChanged)
    Q_PROPERTY(quint32 width READ width WRITE setWidth NOTIFY widthChanged)
    Q_PROPERTY(QQmlListProperty<QPointField> fields READ fields NOTIFY fieldsChanged)
    Q_PROPERTY(quint8 is_bigendian READ is_bigendian WRITE setIs_bigendian NOTIFY is_bigendianChanged)
    Q_PROPERTY(quint32 point_step READ point_step WRITE setPoint_step NOTIFY point_stepChanged)
    Q_PROPERTY(quint32 row_step READ row_step WRITE setRow_step NOTIFY row_stepChanged)
    Q_PROPERTY(QByteArray data READ data WRITE setData NOTIFY dataChanged)
    Q_PROPERTY(quint8 is_dense READ is_dense WRITE setIs_dense NOTIFY is_denseChanged)
    Q_PROPERTY(QVector3D minimum READ minimum NOTIFY minimumChanged)
    Q_PROPERTY(QVector3D maximum READ maximum NOTIFY maximumChanged)
    Q_PROPERTY(QVector3D centroid READ centroid NOTIFY centroidChanged)
    Q_PROPERTY(QVector3D offset READ offset NOTIFY offsetChanged)

public:
    QPointCloud(QPointCloud *copy);
    QPointCloud(QObject *parent = nullptr);

    QPointCloud(pcl::PCLPointCloud2 *pointCloud);
    ~QPointCloud();

    void loadPointCloud(const QString& filePath);
    void updateAttributes();

    quint32 height() const;
    quint32 width() const;

    QQmlListProperty<QPointField> fields();

    quint8 is_bigendian() const;
    quint32 point_step() const;
    quint32 row_step() const;
    QByteArray data() const;
    quint8 is_dense() const;
    QVector3D minimum() const;
    QVector3D maximum() const;
    QVector3D centroid() const;
    QVector3D offset() const;

    const QList<QPointField *> &getFields();

    pcl::PCLPointCloud2* pointCloud();
    void setPointCloud(const pcl::PCLPointCloud2 &copy);

    QVector<QVector3D> vertices() const { return m_points; }
    QVector<QVector3D> colors() const { return m_colors; }
    QVector<QVector3D> normals() const { return m_normals; }
    int pointsNumber() const { return m_pointsCount; }

public slots:
    void setHeight(quint32 height);
    void setWidth(quint32 width);
    void setIs_bigendian(quint8 is_bigendian);
    void setPoint_step(quint32 point_step);
    void setRow_step(quint32 row_step);
    void setData(QByteArray data);
    void setIs_dense(quint8 is_dense);

signals:
    void heightChanged(quint32 height);
    void widthChanged(quint32 width);
    void fieldsChanged(QQmlListProperty<QPointField> fields);
    void is_bigendianChanged(quint8 is_bigendian);
    void point_stepChanged(quint32 point_step);
    void row_stepChanged(quint32 row_step);
    void dataChanged(QByteArray data);
    void is_denseChanged(quint8 is_dense);
    void minimumChanged(QVector3D minimum);
    void maximumChanged(QVector3D maximum);
    void centroidChanged(QVector3D centroid);
    void offsetChanged(QVector3D offset);

private:

    QPointCloudLoader *m_loader;
    QPointCloudPrivate  *m_priv;

    QVector<QVector3D> m_points;
    QVector<QVector3D> m_colors;
    QVector<QVector3D> m_normals;
    int m_pointsCount;

};

#endif // QPOINTCLOUD_H
