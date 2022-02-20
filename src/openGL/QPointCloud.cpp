#include "openGL/QPointCloud.h"
#include "Constant.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <QRgb>
#include <QFile>
#include <QVector>
#include <QDebug>

#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

class QPointCloudPrivate
{
public:
    QPointCloudPrivate(QPointCloud* p)
        : m_parent(p)
        , m_pointcloud(nullptr)
        , m_height(1)
        , m_width(0)
        , m_is_bigendian(0)
        , m_point_step(0)
        , m_row_step(0)
        , m_data()
        , m_is_dense(0)
        , m_minimum()
        , m_maximum()
        , m_centroid()
        , m_offset()
        , m_dirtyMinMax(true)
        , m_dirtyCentroid(true)
    {}
    QPointCloud *m_parent;
    pcl::PCLPointCloud2 *m_pointcloud;
    QList<QPointField*> m_fields;

    quint32 m_height;
    quint32 m_width;

    quint8 m_is_bigendian;
    quint32 m_point_step;
    quint32 m_row_step;
    QByteArray m_data;
    quint8 m_is_dense;

    QVector3D m_minimum;
    QVector3D m_maximum;
    QVector3D m_centroid;
    QVector3D m_offset;
    bool m_dirtyMinMax;
    bool m_dirtyCentroid;

    static void fields_append(QQmlListProperty<QPointField> *self, QPointField* f);
    static int fields_count(QQmlListProperty<QPointField> *self);
    static QPointField* fields_at(QQmlListProperty<QPointField> *self, int i);
    static void fields_clear(QQmlListProperty<QPointField> *self);

    void updateMinMax()
    {
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromPCLPointCloud2(*m_pointcloud, pc);
        pcl::getMinMax3D(pc, min, max);
        m_minimum = QVector3D(min.x, min.y, min.z);
        m_maximum = QVector3D(max.x, max.y, max.z);
        m_dirtyMinMax = false;

    }
    void updateCentroid()
    {

    }

};

QPointCloud::QPointCloud(QObject *parent)
    : QObject{parent}
    , m_priv(new QPointCloudPrivate(this))
{

}

QPointCloud::QPointCloud(QPointCloud *copy)
    :m_priv(new QPointCloudPrivate(this))
{
    this->setPointCloud( *copy->m_priv->m_pointcloud );
}

QPointCloud::QPointCloud(pcl::PCLPointCloud2 *pointcloud)
    :m_priv(new QPointCloudPrivate(this))
{
    m_priv->m_pointcloud = pointcloud;
}

QPointCloud::~QPointCloud()
{
    if(m_priv->m_pointcloud != nullptr)
    {
        delete m_priv->m_pointcloud;
    }
    delete m_priv;
}

void QPointCloud::loadPointCloud(const QString &filePath)
{

    CONSOLE << filePath;
    pcl::PointCloud<pcl::PointXYZRGB> pcd;

    if(filePath.endsWith(".pcd", Qt::CaseInsensitive))
    {
        pcl::PCDReader reader;

        reader.read(filePath.toStdString(), pcd);

        CONSOLE << "Fucking PCL";

        // pcl::toPCLPointCloud2(pcd, *m_priv->m_pointcloud);
    }
    else if(filePath.endsWith(".ply", Qt::CaseInsensitive))
    {
        pcl::PLYReader reader;
        reader.read(filePath.toStdString(), *m_priv->m_pointcloud);
    }
    qDebug() << "Read Pointcloud" << filePath << "with" << pcd.size() << "points.";

    // TODO: m_points, m_colors, m_normals;
    pcl::PointCloud<pcl::PointXYZRGB> pointCloudWithColor;
    // pcl::fromPCLPointCloud2(*m_priv->m_pointcloud, pointCloudWithColor);

    m_points.clear();
    m_colors.clear();
    for (size_t i = 0; i < pcd.size(); i++){
        m_points.append(QVector3D(pcd.at(i).x, pcd.at(i).y, pcd.at(i).z));
        m_colors.append(QVector3D(pcd.at(i).r, pcd.at(i).g, pcd.at(i).b));
    }

    assert(m_points.size() == m_colors.size());
    m_pointsCount = pcd.size();
    CONSOLE << "Reading done";
}

void QPointCloud::updateAttributes()
{
    if(m_priv->m_pointcloud)
    {
        m_priv->m_fields.clear();
        for(int i=0 ; m_priv->m_pointcloud->fields.size() > i; ++i)
        {
            pcl::PCLPointField &pf( m_priv->m_pointcloud->fields[i] );
            m_priv->m_fields.append(new QPointField(&pf));
        }
    }
}

quint32 QPointCloud::height() const
{
    if(m_priv->m_pointcloud)
    {
        return m_priv->m_pointcloud->height;
    }
    else{
        return m_priv->m_height;
    }
}

quint32 QPointCloud::width() const
{
    if(m_priv->m_pointcloud)
    {
        return m_priv->m_pointcloud->width;
    }
    else
    {
        return m_priv->m_width;
    }
}

QQmlListProperty<QPointField> QPointCloud::fields()
{
    return QQmlListProperty<QPointField>(this, static_cast<void*>(m_priv), &QPointCloudPrivate::fields_append, &QPointCloudPrivate::fields_count, &QPointCloudPrivate::fields_at, &QPointCloudPrivate::fields_clear);
}

void QPointCloudPrivate::fields_append(QQmlListProperty<QPointField> *self, QPointField *f)
{
    QPointCloudPrivate *that = static_cast<QPointCloudPrivate*>(self->data);
    if(that->m_pointcloud)
    {
        // This is not typical for PCL. PCL would create a new pointcloud instead.
        // if this is needed, there should be a conversion from PCL Pointcloud to a
        // custom pointcloud format.
        Q_ASSERT_X(false, "QPointcloud::fields_append", "Must not be called.");
        pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(that->m_pointcloud);
        p->fields.push_back(*f->getPointField());
    }
    else
    {
        that->m_fields.append(f);
    }
}

int QPointCloudPrivate::fields_count(QQmlListProperty<QPointField> *self)
{
    QPointCloudPrivate *that = static_cast<QPointCloudPrivate*>(self->data);
    if(that->m_pointcloud)
    {
        Q_ASSERT_X(that->m_fields.count() == 0, "QPointcloudPrivate::fields_count", "Mixed up pcl and non pcl.");
        pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(that->m_pointcloud);
        return p->fields.size();
    }
    else
    {
        that->m_fields.count();
    }
}

QPointField *QPointCloudPrivate::fields_at(QQmlListProperty<QPointField> *self, int i)
{
    QPointCloudPrivate *that = static_cast<QPointCloudPrivate*>(self->data);
    if(that->m_pointcloud)
    {
        Q_ASSERT_X(that->m_fields.count() == 0, "QPointcloudPrivate::fields_count", "Mixed up pcl and non pcl.");
        pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(that->m_pointcloud);
        return new QPointField(&p->fields.at(i));
    }
    else
    {
        that->m_fields.at(i);
    }
}

void QPointCloudPrivate::fields_clear(QQmlListProperty<QPointField> *self)
{
    QPointCloudPrivate *that = static_cast<QPointCloudPrivate*>(self->data);
    if(that->m_pointcloud)
    {
        Q_ASSERT_X(that->m_fields.count() == 0, "QPointcloudPrivate::fields_count", "Mixed up pcl and non pcl.");
        pcl::PCLPointCloud2 *p = static_cast<pcl::PCLPointCloud2*>(that->m_pointcloud);
        p->fields.clear();
    }
    else
    {
        that->m_fields.clear();
    }
}

quint8 QPointCloud::is_bigendian() const
{
    if(m_priv->m_pointcloud)
    {
        return m_priv->m_pointcloud->is_bigendian;
    }
    else
    {
        return m_priv->m_is_bigendian;
    }
}

quint32 QPointCloud::point_step() const
{
    if(m_priv->m_pointcloud)
    {
        return m_priv->m_pointcloud->point_step;
    }
    else
    {
        return m_priv->m_point_step;
    }
}

quint32 QPointCloud::row_step() const
{
    if(m_priv->m_pointcloud)
    {
        return m_priv->m_pointcloud->row_step;
    }
    else
    {
        return m_priv->m_row_step;
    }
}

QByteArray QPointCloud::data() const
{
    if(m_priv->m_pointcloud)
    {
        return QByteArray(reinterpret_cast<char*>(&m_priv->m_pointcloud->data[0]), m_priv->m_pointcloud->data.size());
        //return QByteArray::fromRawData(reinterpret_cast<const char*>(&m_priv->m_pointcloud->data[0]), m_priv->m_pointcloud->data.size());
    }
    else
    {
        return m_priv->m_data;
    }
}

quint8 QPointCloud::is_dense() const
{
    if(m_priv->m_pointcloud)
    {
        return m_priv->m_pointcloud->is_dense;
    }
    else
    {
        return m_priv->m_is_dense;
    }
}

const QList<QPointField *> &QPointCloud::getFields()
{
    return m_priv->m_fields;
}

pcl::PCLPointCloud2 *QPointCloud::pointCloud()
{
    if(nullptr == m_priv->m_pointcloud)
    {
        m_priv->m_pointcloud = new pcl::PCLPointCloud2();
    }
    return m_priv->m_pointcloud;
}

void QPointCloud::setPointCloud(const pcl::PCLPointCloud2& copy)
{
    if(m_priv->m_pointcloud != nullptr)
    {
        delete m_priv->m_pointcloud;
    }
    m_priv->m_pointcloud = new pcl::PCLPointCloud2(copy);
}

QVector3D QPointCloud::minimum() const
{
    if(m_priv->m_dirtyMinMax)
    {
        m_priv->updateMinMax();
    }
    return m_priv->m_minimum;
}

QVector3D QPointCloud::maximum() const
{
    if(m_priv->m_dirtyMinMax)
    {
        m_priv->updateMinMax();
    }
    return m_priv->m_maximum;
}

QVector3D QPointCloud::centroid() const
{
    if(m_priv->m_dirtyCentroid)
    {
        m_priv->updateCentroid();
    }
    return m_priv->m_centroid;
}

QVector3D QPointCloud::offset() const
{
    return m_priv->m_offset;
}

void QPointCloud::setHeight(quint32 height)
{
    if(m_priv->m_pointcloud)
    {
        if (m_priv->m_pointcloud->height == height)
            return;
        m_priv->m_pointcloud->height = height;
    }
    else
    {
        if (m_priv->m_height == height)
            return;
        m_priv->m_height = height;
    }
    emit heightChanged(height);
}

void QPointCloud::setWidth(quint32 width)
{
    if(m_priv->m_pointcloud)
    {
        if (m_priv->m_pointcloud->width == width)
            return;
        m_priv->m_pointcloud->width = width;
    }
    else
    {
        if (m_priv->m_width == width)
            return;
        m_priv->m_width = width;
    }
    emit widthChanged(width);
}

void QPointCloud::setIs_bigendian(quint8 is_bigendian)
{
    if(m_priv->m_pointcloud)
    {
        if (m_priv->m_pointcloud->is_bigendian == is_bigendian)
            return;
        m_priv->m_pointcloud->is_bigendian = is_bigendian;
    }
    else
    {
        if (m_priv->m_is_bigendian == is_bigendian)
            return;
        m_priv->m_is_bigendian = is_bigendian;
    }
    emit is_bigendianChanged(is_bigendian);
}

void QPointCloud::setPoint_step(quint32 point_step)
{
    if(m_priv->m_pointcloud)
    {
        if (m_priv->m_pointcloud->point_step == point_step)
            return;
        m_priv->m_pointcloud->point_step = point_step;
    }
    else
    {
        if (m_priv->m_point_step == point_step)
            return;
        m_priv->m_point_step = point_step;
    }

    emit point_stepChanged(point_step);
}

void QPointCloud::setRow_step(quint32 row_step)
{
    if(m_priv->m_pointcloud)
    {
        if (m_priv->m_pointcloud->row_step == row_step)
            return;

        m_priv->m_pointcloud->row_step = row_step;
    }
    else
    {
        if (m_priv->m_row_step == row_step)
            return;
        m_priv->m_row_step = row_step;
    }
    emit row_stepChanged(row_step);
}

void QPointCloud::setData(QByteArray data)
{
    if(m_priv->m_pointcloud)
    {
        m_priv->m_pointcloud->data.resize(data.size());
        memcpy(&m_priv->m_pointcloud->data[0], data.data(), m_priv->m_pointcloud->data.size());
    }
    else
    {
        m_priv->m_data = data;
    }
    emit dataChanged(data);
}

void QPointCloud::setIs_dense(quint8 is_dense)
{
    if(m_priv->m_pointcloud)
    {
        if (m_priv->m_pointcloud->is_dense == is_dense)
            return;

        m_priv->m_pointcloud->is_dense = is_dense;
    }
    else
    {
        if (m_priv->m_is_dense == is_dense)
            return;
        m_priv->m_is_dense = is_dense;
    }
    emit is_denseChanged(is_dense);
}
