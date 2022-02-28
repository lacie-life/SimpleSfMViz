#include "openGL/QOpenGLCamera.h"
#include "AppConstant.h"

QOpenGLCamera::QOpenGLCamera(QObject *parent)
    : QObject(parent)
    , m_azimuth(0.0)
    , m_elevation(15.0)
    , m_distance(15.0)
{
}

float QOpenGLCamera::azimuth() const
{
    return m_azimuth;
}

float QOpenGLCamera::distance() const
{
    return m_distance;
}

float QOpenGLCamera::elevation() const
{
    return m_elevation;
}

void QOpenGLCamera::setAzimuth(float azimuth)
{
    if (m_azimuth == azimuth)
        return;

    m_azimuth = azimuth;
    emit azimuthChanged(azimuth);
}

void QOpenGLCamera::setDistance(float distance)
{
    if (m_distance == distance)
        return;

    m_distance = distance;
    emit distanceChanged(distance);
}

void QOpenGLCamera::setElevation(float elevation)
{
    if (m_elevation == elevation)
        return;

    m_elevation = elevation;
    emit elevationChanged(elevation);
}


