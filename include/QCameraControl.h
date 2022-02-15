#ifndef QCAMERACONTROL_H
#define QCAMERACONTROL_H

#include <QObject>
#include <QVector3D>

struct QCameraState {
    QCameraState(const QVector3D& position_, const QVector3D& rotation_,
                 double frontClippingDistance_, double farClippingDistance_)
        : position(position_),
          rotation(rotation_),
          frontClippingDistance(frontClippingDistance_),
          rearClippingDistance(farClippingDistance_)
    {}

    QVector3D position;
    QVector3D rotation;
    double frontClippingDistance;
    double rearClippingDistance;
};

class QCameraControl : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QCameraState currentState READ currentState WRITE setCurrentState NOTIFY currentStateChanged)

public:
    enum RotationSTEP {RK = 1};

    QCameraControl(QObject *parent = nullptr);

    void forward();
    void backward();
    void left();
    void right();
    void up();
    void down();
    void setPosition(const QVector3D& position);

    void rotate(int dx, int dy, int dz);

    void setFrontCPDistance(double distance);
    void setRearCPDistance(double distance);

    QCameraState currentState() const;


signals:
    void changed(const QCameraState& newState);
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

    void currentStateChanged(QCameraState state);


public slots:
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);

    void setCurrentState(QCameraState state);


private:
    QVector3D m_position;
    int m_xRotation;
    int m_yRotation;
    int m_zRotation;

    double m_frontClippingPlaneDistance;
    double m_rearClippingDistance;

    void notify() {emit currentStateChanged(currentState());}

};

#endif // QCAMERACONTROL_H
