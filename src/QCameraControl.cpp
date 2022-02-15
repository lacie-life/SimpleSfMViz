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
  notify();
}


void QCameraControl::backward()
{
  m_position[2] -= CAMERA_STEP;
  notify();
}


void QCameraControl::left()
{
  m_position[0] += CAMERA_STEP;
  notify();
}


void QCameraControl::right()
{
  m_position[0] -= CAMERA_STEP;
  notify();

}


void QCameraControl::up()
{
  m_position[1] -= CAMERA_STEP;
  notify();
}


void QCameraControl::down()
{
  m_position[1] += CAMERA_STEP;
  notify();

}


void QCameraControl::setFrontCPDistance(double distance) {
  m_frontClippingPlaneDistance = distance;
  notify();
}


void QCameraControl::setRearCPDistance(double distance) {
  m_rearClippingDistance = distance;
  notify();
}


void QCameraControl::setPosition(const QVector3D& position) {
  m_position = position;
}


void QCameraControl::setXRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != m_xRotation) {
    m_xRotation = angle;
    emit xRotationChanged(angle);
    notify();
  }
}


void QCameraControl::setYRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != m_yRotation) {
    m_yRotation = angle;
    emit yRotationChanged(angle);
    notify();
  }
}


void QCameraControl::setZRotation(int angle)
{
  angle = angle % (360 * RK);
  if (angle != m_zRotation) {
    m_zRotation = angle;
    emit zRotationChanged(angle);
    notify();
  }
}

void QCameraControl::setCurrentState(QCameraState state)
{
    m_position = state.position;
    m_xRotation = state.rotation.x();
    m_yRotation = state.rotation.y();
    m_zRotation = state.rotation.z();
    m_frontClippingPlaneDistance = state.frontClippingDistance;
    m_rearClippingDistance = state.rearClippingDistance;

    notify();
}


void QCameraControl::rotate(int dx, int dy, int dz) {
  setXRotation(m_xRotation + dx);
  setYRotation(m_yRotation + dy);
  setZRotation(m_zRotation + dz);
}


QCameraState QCameraControl::currentState() const {
  return QCameraState(
    m_position,
    QVector3D((float)m_xRotation/RK, (float)m_yRotation/RK, (float)m_zRotation/RK),
    m_frontClippingPlaneDistance,
    m_rearClippingDistance
    );
}

