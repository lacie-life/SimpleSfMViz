#ifndef QCAMERACONTROL_H
#define QCAMERACONTROL_H

#include <QObject>
#include <QVector3D>

class QCameraControl : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float azimuth READ azimuth WRITE setAzimuth NOTIFY azimuthChanged)
    Q_PROPERTY(float elevation READ elevation WRITE setElevation NOTIFY elevationChanged)
    Q_PROPERTY(float distance READ distance WRITE setDistance NOTIFY distanceChanged)

public:
    explicit QCameraControl(QObject *parent = 0);

    float azimuth() const;
    float distance() const;
    float elevation() const;

signals:
    void azimuthChanged(float azimuth);
    void distanceChanged(float distance);
    void elevationChanged(float elevation);

public slots:
    void setAzimuth(float azimuth);
    void setDistance(float distance);
    void setElevation(float elevation);

private:
    float m_azimuth;
    float m_elevation;
    float m_distance;

};

#endif // QCAMERACONTROL_H
