#include "pcl/QPointField.h"

#include <pcl/PCLPointField.h>

pcl::uint8_t toPCLDataType(const QPointField::PointFieldTypes& datatype)
{
    switch(datatype)
    {
    case QPointField::INT8:
        return pcl::PCLPointField::INT8;
    case QPointField::UINT8:
        return pcl::PCLPointField::UINT8;
    case QPointField::INT16:
        return pcl::PCLPointField::INT16;
    case QPointField::UINT16:
        return pcl::PCLPointField::UINT16;
    case QPointField::INT32:
        return pcl::PCLPointField::INT32;
    case QPointField::UINT32:
        return pcl::PCLPointField::UINT32;
    case QPointField::FLOAT32:
        return pcl::PCLPointField::FLOAT32;
    case QPointField::FLOAT64:
        return pcl::PCLPointField::FLOAT64;
    }
    Q_ASSERT_X(false, "fromPCLDatatype", "unknown PCL pointfield");
    return pcl::PCLPointField::INT8;
}

QPointField::PointFieldTypes fromPCLDataType(const pcl::uint8_t& datatype)
{
    switch(datatype)
    {
    case pcl::PCLPointField::INT8:
        return QPointField::INT8;
    case pcl::PCLPointField::UINT8:
        return QPointField::UINT8;
    case pcl::PCLPointField::INT16:
        return QPointField::INT16;
    case pcl::PCLPointField::UINT16:
        return QPointField::UINT16;
    case pcl::PCLPointField::INT32:
        return QPointField::INT32;
    case pcl::PCLPointField::UINT32:
        return QPointField::UINT32;
    case pcl::PCLPointField::FLOAT32:
        return QPointField::FLOAT32;
    case pcl::PCLPointField::FLOAT64:
        return QPointField::FLOAT64;
    }
    Q_ASSERT_X(false, "toPCLDatatype", "unknown pointfield");
    return QPointField::INT8;
}

QPointField::QPointField(QObject *parent, QString name, quint32 offset, PointFieldTypes type, quint32 count)
    :QObject(parent)
    ,m_name(name)
    ,m_offset(offset)
    ,m_datatype(type)
    ,m_count(count)
    ,m_pointfield(nullptr)
{
}

QPointField::QPointField(QObject *parent, pcl::PCLPointField *field)
    :QObject(parent)
    ,m_name(field->name.c_str())
    ,m_offset(field->offset)
    ,m_datatype(fromPCLDataType(field->datatype))
    ,m_count(field->count)
    ,m_pointfield(field)
{
}

QPointField::QPointField(pcl::PCLPointField *field)
    :m_name(field->name.c_str())
    ,m_offset(field->offset)
    ,m_datatype(fromPCLDataType(field->datatype))
    ,m_count(field->count)
    ,m_pointfield(field)
{
}

QString QPointField::name() const
{
    return m_name;
}

quint32 QPointField::offset() const
{
    return m_offset;
}

QPointField::PointFieldTypes QPointField::datatype() const
{
    return m_datatype;
}

quint32 QPointField::count() const
{
    return m_count;
}

void QPointField::setName(QString name)
{
    if (m_name == name)
        return;
    m_name = name;

    if(m_pointfield)
    {
        std::string stdname(name.toStdString());
        m_pointfield->name = stdname;
    }

    emit nameChanged(name);
}

void QPointField::setOffset(quint32 offset)
{
    if (m_offset == offset)
        return;
    m_offset = offset;
    if(m_pointfield)
    {
        m_pointfield->offset = offset;
    }
    emit offsetChanged(offset);
}

void QPointField::setDatatype(QPointField::PointFieldTypes datatype)
{
    if (m_datatype == datatype)
        return;
    m_datatype = datatype;

    if(m_pointfield)
    {
        m_pointfield->datatype = toPCLDataType(datatype);
    }

    emit datatypeChanged(datatype);
}

void QPointField::setCount(quint32 count)
{
    if (m_count == count)
        return;
    m_count = count;
    if(m_pointfield)
    {
        m_pointfield->count = count;
    }
    emit countChanged(count);
}


