#ifndef QCAMERACONTROL_H
#define QCAMERACONTROL_H

#include <QObject>
#include <QVector3D>

class QCameraControl : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVector3D position READ position WRITE setPosition NOTIFY positionChanged)

    Q_PROPERTY(int xRotation READ xRotation WRITE setxRotation NOTIFY xRotationChanged)
    Q_PROPERTY(int yRotation READ yRotation WRITE setyRotation NOTIFY yRotationChanged)
    Q_PROPERTY(int zRotation READ zRotation WRITE setzRotation NOTIFY zRotationChanged)

    Q_PROPERTY(double frontClippingPlaneDistance READ frontClippingPlaneDistance WRITE setFrontClippingPlaneDistance NOTIFY frontClippingPlaneDistanceChanged)
    Q_PROPERTY(double rearClippingDistance READ rearClippingDistance WRITE setRearClippingDistance NOTIFY rearClippingDistanceChanged)

public:
    enum RotationSTEP {RK = 1};

    const float CAMERA_STEP = 0.005;

    QCameraControl(QObject *parent = nullptr);

    void forward();
    void backward();
    void left();
    void right();
    void up();
    void down();

    void rotate(int dx, int dy, int dz);

    QVector3D position() const;

    int xRotation() const;
    int yRotation() const;
    int zRotation() const;

    double frontClippingPlaneDistance() const;
    double rearClippingDistance() const;


signals:
    void positionChanged(QVector3D position);

    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

    void frontClippingPlaneDistanceChanged(double distance);
    void rearClippingDistanceChanged(double distance);



public slots:
    void setPosition(QVector3D position);

    void setxRotation(int angle);
    void setyRotation(int angle);
    void setzRotation(int angle);

    void setFrontClippingPlaneDistance(double distance);
    void setRearClippingDistance(double distance);

private:
    QVector3D m_position;
    int m_xRotation;
    int m_yRotation;
    int m_zRotation;

    double m_frontClippingPlaneDistance;
    double m_rearClippingDistance;

};

#endif // QCAMERACONTROL_H
