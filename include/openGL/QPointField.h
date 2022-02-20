#ifndef QPOINTFIELD_H
#define QPOINTFIELD_H

#include <QObject>

namespace pcl {
    class PCLPointField;
}

class QPointField : public QObject
{
    Q_OBJECT
    Q_ENUMS(PointFieldTypes)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(quint32 offset READ offset WRITE setOffset NOTIFY offsetChanged)
    Q_PROPERTY(PointFieldTypes datatype READ datatype WRITE setDatatype NOTIFY datatypeChanged)
    Q_PROPERTY(quint32 count READ count WRITE setCount NOTIFY countChanged)

public:

    enum PointFieldTypes{
        INT8,
        UINT8,
        INT16,
        UINT16,
        INT32,
        UINT32,
        FLOAT32,
        FLOAT64
    };

    QPointField(QObject *parent, QString name, quint32 offset, PointFieldTypes type, quint32 count);
    QPointField(QObject *parent, pcl::PCLPointField *field);
    QPointField(pcl::PCLPointField *field);

    QString name() const;
    quint32 offset() const;
    PointFieldTypes datatype() const;
    quint32 count() const;
    const pcl::PCLPointField* getPointField();

public slots:
    void setName(QString name);
    void setOffset(quint32 offset);
    void setDatatype(QPointField::PointFieldTypes datatype);
    void setCount(quint32 count);

signals:
    void nameChanged(QString name);
    void offsetChanged(quint32 offset);
    void datatypeChanged(QPointField::PointFieldTypes datatype);
    void countChanged(quint32 count);

private:
    QString m_name;
    quint32 m_offset;
    PointFieldTypes m_datatype;
    quint32 m_count;
    pcl::PCLPointField *m_pointfield;
};

#endif // QPOINTFIELD_H
