#ifndef QPOINTCLOUDRENDERER_H
#define QPOINTCLOUDRENDERER_H

#include <QObject>
#include <QScopedPointer>
#include <QSharedPointer>
#include <QMatrix4x4>

#include "QCameraControl.h"
#include "QPointCloud.h"
#include "QPointCloudLoader.h"

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
    void initialize(const QString& filePath);
    void render();
    void invalidate();

    void setPosition(QVector3D position);
    void setMVP(QMatrix4x4 mat);

    void setxRotation(int angle);
    void setyRotation(int angle);
    void setzRotation(int angle);

    void setFrontClippingPlaneDistance(double distance);
    void setRearClippingDistance(double distance);

private:
    void loadPLY(const QString& plyFilePath);

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

    QPointCloud* m_pointCloud;
    unsigned int positionNumberIndicies;
    QScopedPointer<QOpenGLBuffer> m_positionsBuffer;
    QScopedPointer<QOpenGLBuffer> m_colorsBuffer;

    QMatrix4x4 m_projectionMatrix;
    QMatrix4x4 m_cameraMatrix;
    QMatrix4x4 m_worldMatrix;

    QMatrix4x4 m_MVP;

    double m_frontClippingPlaneDistance;
    double m_rearClippingDistance;
    QVector3D m_position;
    int m_xRotation;
    int m_yRotation;
    int m_zRotation;
};

#endif // QPOINTCLOUDRENDERER_H
