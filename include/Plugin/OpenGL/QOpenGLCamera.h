#ifndef QOPENGLCAMERA_H
#define QOPENGLCAMERA_H

#include <QObject>

const float ELEVATION_STEP = 1.0;
const float AZIMUTH_STEP = 1.0;
const float DISTANCE_STEP = 0.1;

class QOpenGLCamera : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float azimuth READ azimuth WRITE setAzimuth NOTIFY azimuthChanged)
    Q_PROPERTY(float elevation READ elevation WRITE setElevation NOTIFY elevationChanged)
    Q_PROPERTY(float distance READ distance WRITE setDistance NOTIFY distanceChanged)

public:

    enum CameraDirection {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD,
    };

    explicit QOpenGLCamera(QObject *parent = 0);

    float azimuth() const;
    float distance() const;
    float elevation() const;

    void move(CameraDirection dir);

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


#endif // QOPENGLCAMERA_H
