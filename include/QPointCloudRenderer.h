#ifndef QPOINTCLOUDRENDERER_H
#define QPOINTCLOUDRENDERER_H

#include <QObject>
#include <QScopedPointer>
#include <QSharedPointer>
#include <QMatrix4x4>

#include "QCameraControl.h"

class QPointCloudRenderer: public QObject
{
    Q_OBJECT
public:

    enum colorAxisMode {COLOR_BY_ROW, COLOR_BY_Z};

    enum CoordinateMirroring {
        DoNotMirrorCoordinates,
        MirrorYCoordinate
    };

    QPointCloudRenderer(QObject *parent = 0);
    ~QPointCloudRenderer();

    // All assume that the GL context is current.
    void initialize(CoordinateMirroring cm = DoNotMirrorCoordinates);
    void render();
    void invalidate();

    void setAzimuth(float azimuth);
    void setElevation(float elevation);
    void setDistance(float distance);

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

    CoordinateMirroring m_coordinateMirroring;

    float m_azimuth;
    float m_elevation;
    float m_distance;
};

#endif // QPOINTCLOUDRENDERER_H
