#include "QCameraControl.h"

const float CAMERA_STEP = 0.01;

QCameraControl::QCameraControl(QObject *parent)
    : QObject{parent}
    , m_xRotation(0)
    , m_yRotation(0)
    , m_zRotation(0)
    , m_frontClippingPlaneDistance(0)
    , m_rearClippingDistance(0)
{

}

void QCameraControl::forward()
{
  m_position[2] += CAMERA_STEP;
}


void QCameraControl::backward()
{
  m_position[2] -= CAMERA_STEP;
}


void QCameraControl::left()
{
  m_position[0] += CAMERA_STEP;
}


void QCameraControl::right()
{
  m_position[0] -= CAMERA_STEP;

}


void QCameraControl::up()
{
  m_position[1] -= CAMERA_STEP;
}


void QCameraControl::down()
{
  m_position[1] += CAMERA_STEP;

}


void QCameraControl::setFrontClippingPlaneDistance(double distance) {
  m_frontClippingPlaneDistance = distance;

  emit frontClippingPlaneDistanceChanged(distance);
}


void QCameraControl::setRearClippingDistance(double distance) {
  m_rearClippingDistance = distance;

  emit rearClippingDistanceChanged(distance);
}


void QCameraControl::setPosition(QVector3D position) {
  m_position = position;

  emit positionChanged(position);
}


void QCameraControl::setxRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != m_xRotation) {
    m_xRotation = angle;
    emit xRotationChanged(angle);
  }
}


void QCameraControl::setyRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != m_yRotation) {
    m_yRotation = angle;
    emit yRotationChanged(angle);
  }
}


void QCameraControl::setzRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != m_zRotation) {
    m_zRotation = angle;
    emit zRotationChanged(angle);
  }
}

void QCameraControl::rotate(int dx, int dy, int dz) {
  setxRotation(m_xRotation + dx);
  setyRotation(m_yRotation + dy);
  setzRotation(m_zRotation + dz);
}

QVector3D QCameraControl::position() const
{
    return m_position;
}

int QCameraControl::xRotation() const
{
    return m_xRotation;
}

int QCameraControl::yRotation() const
{
    return m_yRotation;
}

int QCameraControl::zRotation() const
{
    return m_zRotation;
}

double QCameraControl::frontClippingPlaneDistance() const
{
    return m_frontClippingPlaneDistance;
}

double QCameraControl::rearClippingDistance() const
{
    return m_rearClippingDistance;
}




