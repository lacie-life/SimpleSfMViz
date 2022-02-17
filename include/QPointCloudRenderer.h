#ifndef QPOINTCLOUDRENDERER_H
#define QPOINTCLOUDRENDERER_H

#include <QObject>
#include <QScopedPointer>
#include <QSharedPointer>
#include <QMatrix4x4>

#include "QCameraControl.h"

class QOpenGLBuffer;
class QOpenGLShaderProgram;
class QOpenGLVertexArrayObject;

class QPointCloudRenderer: public QObject
{
    Q_OBJECT
public:

    enum colorAxisMode {COLOR_BY_ROW, COLOR_BY_Z};

    QPointCloudRenderer(QObject *parent = 0);
    ~QPointCloudRenderer();

    // All assume that the GL context is current.
    void initialize(const QString& plyFilePath);
    void render();
    void invalidate();

    void cameraForward();
    void cameraBackward();
    void cameraLeft();
    void cameraRight();
    void cameraUp();
    void cameraDown();

    void setPosition(QVector3D position);

    void setxRotation(int angle);
    void setyRotation(int angle);
    void setzRotation(int angle);

    void setFrontClippingPlaneDistance(double distance);
    void setRearClippingDistance(double distance);

private:
    void loadPLY(const QString& plyFilePath);
    void drawFrameAxis();

private:
    float m_pointSize;
    colorAxisMode m_colorMode;
    std::vector<std::pair<QVector3D, QColor> > m_axesLines;

    QScopedPointer<QOpenGLVertexArrayObject> m_vao;
    QScopedPointer<QOpenGLBuffer> m_vertexBuffer;
    QScopedPointer<QOpenGLShaderProgram> m_shaders;

    QVector<float> m_pointsData;
    size_t m_pointsCount;
    QVector3D m_pointsBoundMin;
    QVector3D m_pointsBoundMax;
    QVector3D m_ray;

    QMatrix4x4 m_projectionMatrix;
    QMatrix4x4 m_cameraMatrix;
    QMatrix4x4 m_worldMatrix;

    double m_frontClippingPlaneDistance;
    double m_rearClippingDistance;
    QVector3D m_position;
    int m_xRotation;
    int m_yRotation;
    int m_zRotation;
};

#endif // QPOINTCLOUDRENDERER_H
