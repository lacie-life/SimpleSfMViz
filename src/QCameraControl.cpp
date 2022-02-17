#include "QCameraControl.h"

QCameraControl::QCameraControl(QObject *parent)
    : QObject(parent)
    , m_azimuth(0.0)
    , m_elevation(15.0)
    , m_distance(15.0)
{
}

float QCameraControl::azimuth() const
{
    return m_azimuth;
}

float QCameraControl::distance() const
{
    return m_distance;
}

float QCameraControl::elevation() const
{
    return m_elevation;
}

void QCameraControl::setAzimuth(float azimuth)
{
    if (m_azimuth == azimuth)
        return;

    m_azimuth = azimuth;
    emit azimuthChanged(azimuth);
}

void QCameraControl::setDistance(float distance)
{
    if (m_distance == distance)
        return;

    m_distance = distance;
    emit distanceChanged(distance);
}

void QCameraControl::setElevation(float elevation)
{
    if (m_elevation == elevation)
        return;

    m_elevation = elevation;
    emit elevationChanged(elevation);
}

