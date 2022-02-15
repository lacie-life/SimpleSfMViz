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

    void setCameraState(const QCameraState& state);

public:
    QSharedPointer<QCameraControl> m_currentCamera;

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
};

#endif // QPOINTCLOUDRENDERER_H
