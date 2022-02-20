#include "pcl/QPointCloudGeometry.h"

#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/qbufferdatagenerator.h>
#include <QHash>
#include <QSharedPointer>

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>

#include <iomanip>

class PointCloudVertexDataGenerator : public Qt3DRender::QBufferDataGenerator /*Qt3DRender::QBufferFunctor*/
{
public:
    PointCloudVertexDataGenerator(QPointCloud *pointcloud)
        : m_bytes(pointcloud->data())
    {
    }

    QByteArray operator ()() Q_DECL_OVERRIDE
    {
        return m_bytes;
    }

    bool operator ==(const Qt3DRender::QBufferDataGenerator &other) const Q_DECL_OVERRIDE
    {
        const PointCloudVertexDataGenerator *otherFunctor = Qt3DRender::functor_cast<PointCloudVertexDataGenerator>(&other);
        if (otherFunctor != NULL)
            return otherFunctor->m_bytes == m_bytes;
        return false;
    }

    QT3D_FUNCTOR(PointCloudVertexDataGenerator)

private:
    QByteArray m_bytes;
};

class PointCloudColorVertexDataGenerator : public Qt3DRender::QBufferDataGenerator /*Qt3DRender::QBufferFunctor*/
{
public:
    PointCloudColorVertexDataGenerator(QPointCloud *pointcloud, bool isUintRgba, int offset)
        : m_pointcloud(new QPointCloud(pointcloud))
        , m_isUintRgba(isUintRgba)
        , m_colorOffset(offset)
    {
    }

    QByteArray operator ()() Q_DECL_OVERRIDE
    {
        int colorChannels = 3;
        int size = m_pointcloud->width() * m_pointcloud->height();
        int pointStep = m_pointcloud->point_step();
        QByteArray data = m_pointcloud->data();
        QByteArray colorData;
        colorData.resize(sizeof(float)*size*colorChannels);
        float *colorDataFloats = reinterpret_cast<float*>(colorData.data());
        for(int i=m_colorOffset ; i<size ; i++) {
            pcl::RGB rgb;
            memcpy (&rgb, &(data.data()[m_colorOffset + i * pointStep]), sizeof (float));
            // qDebug() << rgb.r << "  " << rgb.g << "  " << rgb.b;
            colorDataFloats[(i*colorChannels)  ] = rgb.r/255.f;
            colorDataFloats[(i*colorChannels)+1] = rgb.g/255.f;
            colorDataFloats[(i*colorChannels)+2] = rgb.b/255.f;
            //colorDataFloats[(i*colorChannels+3)] = rgb.a/255.f;
        }
        return colorData;
    }

    bool operator ==(const Qt3DRender::QBufferDataGenerator &other) const Q_DECL_OVERRIDE
    {
        const PointCloudColorVertexDataGenerator *otherFunctor = Qt3DRender::functor_cast<PointCloudColorVertexDataGenerator>(&other);
        if (otherFunctor != NULL)
            return otherFunctor->m_pointcloud == m_pointcloud;
        return false;
    }

    QT3D_FUNCTOR(PointCloudVertexDataGenerator)

private:
    QSharedPointer<QPointCloud> m_pointcloud;
    bool m_isUintRgba;
    int m_colorOffset;
};

class QPointCloudGeometryPrivate
{
public:
    QPointCloudGeometryPrivate()
        :m_vertexBuffer(NULL)
        ,m_colorBuffer(NULL)
        ,m_pointcloud(NULL)
        ,m_colorOffset(0)
        ,m_colorFormatUintRgba(false)
        ,m_colorAvailable(false)
    {}
    Qt3DRender::QBuffer *m_vertexBuffer;
    Qt3DRender::QBuffer *m_colorBuffer;
    QPointCloud *m_pointcloud;
    int m_colorOffset;
    bool m_colorFormatUintRgba;
    bool m_colorAvailable;
};

QPointCloudGeometry::QPointCloudGeometry(Qt3DCore::QNode *parent)
    :m_p(new QPointCloudGeometryPrivate)
{
    m_p->m_vertexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, this);
    m_p->m_colorBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, this);
}

QPointCloudGeometry::~QPointCloudGeometry()
{
    delete m_p;
}

Qt3DRender::QAttribute::VertexBaseType pclTypeToAttributeType(const QPointField::PointFieldTypes &inp)
{
    switch(inp)
    {
    case QPointField::INT8:
        return Qt3DRender::QAttribute::Byte;
    case QPointField::INT16:
        return Qt3DRender::QAttribute::Short;
    case QPointField::INT32:
        return Qt3DRender::QAttribute::Int;
    case QPointField::UINT8:
        return Qt3DRender::QAttribute::UnsignedByte;
    case QPointField::UINT16:
        return Qt3DRender::QAttribute::UnsignedShort;
    case QPointField::UINT32:
        return Qt3DRender::QAttribute::UnsignedInt;
    case QPointField::FLOAT32:
        return Qt3DRender::QAttribute::Float;
    case QPointField::FLOAT64:
        return Qt3DRender::QAttribute::Double;
    default:
        Q_ASSERT(false);
        return Qt3DRender::QAttribute::Float;
    }
}

void QPointCloudGeometry::updateVertices()
{
    if((  m_p->m_pointcloud == NULL
           || m_p->m_pointcloud->pointCloud() == NULL
           || m_p->m_pointcloud->pointCloud()->data.size() == 0)
       &&
           (  m_p->m_pointcloud->data().length() == 0))
        return;

    updateAttributes();
    //QMetaObject::invokeMethod(this, "updateAttributes", Qt::QueuedConnection);
    m_p->m_vertexBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PointCloudVertexDataGenerator(this->m_p->m_pointcloud)));
    if(m_p->m_colorAvailable) {
        m_p->m_colorBuffer->setDataGenerator(Qt3DRender::QBufferDataGeneratorPtr(new PointCloudColorVertexDataGenerator(this->m_p->m_pointcloud, m_p->m_colorFormatUintRgba, m_p->m_colorOffset)));
    }
}

QPointCloud *QPointCloudGeometry::pointCloud() const
{
    return m_p->m_pointcloud;
}

void QPointCloudGeometry::updateAttributes()
{
    // completely rebuild attribute list and remove all previous attributes
    QVector<Qt3DRender::QAttribute *> atts = attributes();
    Q_FOREACH(Qt3DRender::QAttribute *attr, atts)
    {
        if(attr->attributeType() == Qt3DRender::QAttribute::VertexAttribute)
        {
            removeAttribute(attr);
            attr->deleteLater();
        }
        else
        {
            qDebug() << "skipped index";
        }
    }

    // Prepare hash table to query attribute names easily
    QHash<QString, QPointField* > pfs;
    m_p->m_pointcloud->updateAttributes();

    const QList<QPointField *> &fieldList = m_p->m_pointcloud->getFields();
    for(auto fieldIter = fieldList.cbegin(); fieldIter != fieldList.cend() ; ++fieldIter)
    {
        pfs.insert((*fieldIter)->name(), *fieldIter);
    }
    m_p->m_colorAvailable = false;

    // parse point fields and make reasonable attributes out of them
    QHash<QString, QPointField* >::const_iterator pf(pfs.find("x"));
    if(pf != pfs.cend())
    {
        int num = 1 + (pfs.contains("y")?1:0) + (pfs.contains("z")?1:0) + (pfs.contains("w")?1:0);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
        attrib->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
        attrib->setDataType(pclTypeToAttributeType((*pf)->datatype()));
        attrib->setDataSize(num);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        attrib->setBuffer(m_p->m_vertexBuffer);
        attrib->setByteStride(m_p->m_pointcloud->point_step());
        attrib->setByteOffset((*pf)->offset());
        attrib->setCount(m_p->m_pointcloud->width() * m_p->m_pointcloud->height());
        addAttribute(attrib);
        setBoundingVolumePositionAttribute(attrib);
    }
    pf = pfs.find("rgb");
    if(pf != pfs.cend())
    {
        qDebug() << "Color is enabled";
        int num = 3;
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
        attrib->setName(Qt3DRender::QAttribute::defaultColorAttributeName());
        attrib->setDataType(Qt3DRender::QAttribute::Float);
        attrib->setDataSize(num);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        attrib->setBuffer(m_p->m_colorBuffer);
        attrib->setByteStride(num * sizeof(float));
        attrib->setByteOffset(0);
        attrib->setCount(m_p->m_pointcloud->width() * m_p->m_pointcloud->height());
        addAttribute(attrib);
        m_p->m_colorOffset = (*pf)->offset();
        m_p->m_colorFormatUintRgba = false;
        m_p->m_colorAvailable = true;
    }
    pf = pfs.find("rgba");
    if(pf != pfs.cend())
    {
        int num = 3;
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
        attrib->setName(Qt3DRender::QAttribute::defaultColorAttributeName());
        attrib->setDataType(Qt3DRender::QAttribute::Float);
        attrib->setDataSize(num);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        attrib->setBuffer(m_p->m_colorBuffer);
        attrib->setByteStride(num * sizeof(float));
        attrib->setByteOffset(0);
        attrib->setCount(m_p->m_pointcloud->width() * m_p->m_pointcloud->height());
        addAttribute(attrib);
        m_p->m_colorOffset = (*pf)->offset();
        m_p->m_colorFormatUintRgba = true;
        m_p->m_colorAvailable = true;
    }
    pf = pfs.find("normal_x");
    if(pf != pfs.cend())
    {
        int num = 1 + (pfs.contains("normal_y")?1:0) + (pfs.contains("normal_z")?1:0) + (pfs.contains("curvature")?1:0);
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
        attrib->setName(Qt3DRender::QAttribute::defaultNormalAttributeName());
        attrib->setDataType(pclTypeToAttributeType((*pf)->datatype()));
        attrib->setDataSize(num);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        attrib->setBuffer(m_p->m_vertexBuffer);
        attrib->setByteStride(m_p->m_pointcloud->point_step());
        attrib->setByteOffset((*pf)->offset());
        attrib->setCount(m_p->m_pointcloud->width() * m_p->m_pointcloud->height());
        addAttribute(attrib);
    }
    pf = pfs.find("intensity");
    if(pf != pfs.cend())
    {
        Qt3DRender::QAttribute* attrib = new Qt3DRender::QAttribute(nullptr);
        attrib->setName("intensity");
        attrib->setDataType(pclTypeToAttributeType((*pf)->datatype()));
        attrib->setDataSize(1);
        attrib->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        attrib->setBuffer(m_p->m_vertexBuffer);
        attrib->setByteStride(m_p->m_pointcloud->point_step());
        attrib->setByteOffset((*pf)->offset());
        attrib->setCount(m_p->m_pointcloud->width() * m_p->m_pointcloud->height());
        addAttribute(attrib);
    }
}

void QPointCloudGeometry::setPointCloud(QPointCloud *pointcloud)
{
    if (m_p->m_pointcloud == pointcloud)
        return;

    m_p->m_pointcloud = pointcloud;
    updateVertices();
    emit pointCloudChanged(pointcloud);
}

