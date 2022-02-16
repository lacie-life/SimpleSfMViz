#include "QPointCloudRenderer.h"
#include "QCameraControl.h"
#include "Constant.h"

#include <QOpenGLContext>
#include <QOpenGLFunctions>

#include <QOpenGLBuffer>
#include <QOpenGLTexture>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>

#include <QMatrix4x4>
#include <QDebug>
#include <QtMath>

#include <cmath>
#include <cassert>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <limits>

const size_t POINT_STRIDE = 4; // x, y, z, index

QPointCloudRenderer::QPointCloudRenderer(QObject *parent)
    : QObject(parent)
    , m_pointSize(1)
    , m_colorMode(COLOR_BY_Z)
    , m_vao(new QOpenGLVertexArrayObject)
    , m_vertexBuffer(new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer))
    , m_shaders()
    , m_coordinateMirroring(DoNotMirrorCoordinates)
    , m_azimuth(0.0)
    , m_elevation(15.0)
    , m_distance(15.0)
{
    // make trivial axes cross
    m_axesLines.push_back(std::make_pair(QVector3D(0.0, 0.0, 0.0), QColor(1.0, 0.0, 0.0)));
    m_axesLines.push_back(std::make_pair(QVector3D(1.0, 0.0, 0.0), QColor(1.0, 0.0, 0.0)));
    m_axesLines.push_back(std::make_pair(QVector3D(0.0, 0.0, 0.0), QColor(0.0, 1.0, 0.0)));
    m_axesLines.push_back(std::make_pair(QVector3D(0.0, 1.0, 0.0), QColor(0.0, 1.0, 0.0)));
    m_axesLines.push_back(std::make_pair(QVector3D(0.0, 0.0, 0.0), QColor(0.0, 0.0, 1.0)));
    m_axesLines.push_back(std::make_pair(QVector3D(0.0, 0.0, 1.0), QColor(0.0, 0.0, 1.0)));

}

QPointCloudRenderer::~QPointCloudRenderer()
{
    invalidate();
}

void QPointCloudRenderer::initialize(CoordinateMirroring cm)
{
    if (m_vao->isCreated())
        return; // already initialized

    m_coordinateMirroring = cm;

    QString plyPath = "/home/lacie/Github/GreenHouseAR/assest/bunny.ply";

    loadPLY(plyPath);

    if (!m_vao->create())
        qFatal("Unable to create VAO");

    m_vao->bind();

    m_vertexBuffer->create();
    m_vertexBuffer->bind();
    m_vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_vertexBuffer->allocate(m_pointsData.constData(), m_pointsData.size() * sizeof (GLfloat));

    m_shaders.reset(new QOpenGLShaderProgram);
    m_shaders->create();
    m_shaders->addCacheableShaderFromSourceFile(QOpenGLShader::Vertex, "/home/lacie/Github/GreenHouseAR/assest/vertex_shader.glsl");
    m_shaders->addCacheableShaderFromSourceFile(QOpenGLShader::Fragment, "/home/lacie/Github/GreenHouseAR/assest/fragment_shader.glsl");
    m_shaders->link();

    m_shaders->bind();
    m_vertexBuffer->bind();

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), reinterpret_cast<void *>(3*sizeof(GLfloat)));

    m_vao->release();
}

void QPointCloudRenderer::render()
{
    QOpenGLFunctions *functions = QOpenGLContext::currentContext()->functions();

    QMatrix4x4 modelMatrix;
    QMatrix4x4 viewMatrix;
    QMatrix4x4 projectionMatrix;

    modelMatrix.rotate(-90, 0, 1, 0);

    const float azimuthInRadians = qDegreesToRadians(m_azimuth);
    const float elevationInRadians = qDegreesToRadians(m_elevation);

    const QVector3D eyePosition(std::cos(elevationInRadians) * std::cos(azimuthInRadians),
                                std::sin(elevationInRadians),
                                -std::cos(elevationInRadians) * std::sin(azimuthInRadians));

    QVector3D upVector = qFuzzyCompare(m_elevation, 90.0f)
            ? QVector3D(-std::cos(azimuthInRadians), 0, std::sin(azimuthInRadians))
            : QVector3D(0, 1, 0);

    viewMatrix.lookAt(eyePosition * m_distance,
                      QVector3D(0, 0, 0),
                      upVector);

    GLint viewportSize[4];
    functions->glGetIntegerv(GL_VIEWPORT, viewportSize);

     projectionMatrix.perspective(30, float(viewportSize[2]) / viewportSize[3], 0.01, 1000);

     switch (m_coordinateMirroring) {
     case QPointCloudRenderer::DoNotMirrorCoordinates:
         break;
     case QPointCloudRenderer::MirrorYCoordinate:
         projectionMatrix.scale(1, -1, 1);
         break;
     }

     const QMatrix4x4 modelViewMatrix = viewMatrix * modelMatrix;
     const QMatrix4x4 modelViewProjectionMatrix = projectionMatrix * modelViewMatrix;

     CONSOLE << modelViewMatrix ;
     CONSOLE << modelViewProjectionMatrix;

     functions->glClearColor(1.0, 1.0, 1.0, 1.0);
     functions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
     functions->glEnable(GL_DEPTH_TEST);

     m_shaders->bind();

     m_shaders->setUniformValue("model_matrix", modelViewMatrix);
     m_shaders->setUniformValue("MVP", modelViewProjectionMatrix);

     m_shaders->setUniformValue("light.position", QVector4D(0.0, 0.0, 0.0, 1.0));
     m_shaders->setUniformValue("light.intensity", QVector3D(1.0, 1.0, 1.0));

     m_shaders->setUniformValue("material.ka", QVector3D(0.1, 0.1, 0.1));
     m_shaders->setUniformValue("material.kd", QVector3D(1.0, 0.1, 0.1));
     m_shaders->setUniformValue("material.ks", QVector3D(1.0, 1.0, 1.0));
     m_shaders->setUniformValue("material.shininess", 32.0f);

     m_shaders->setUniformValue("pointsCount", static_cast<GLfloat>(m_pointsCount));
     m_shaders->setUniformValue("pointSize", m_pointSize);
     m_shaders->setUniformValue("colorAxisMode", static_cast<GLfloat>(m_colorMode));
     m_shaders->setUniformValue("pointsBoundMin", m_pointsBoundMin);
     m_shaders->setUniformValue("pointsBoundMax", m_pointsBoundMax);

     glDrawArrays(GL_POINTS, 0, m_pointsData.size());
     m_shaders->release();
}


void QPointCloudRenderer::invalidate()
{
    m_vertexBuffer->destroy();
    m_vao->destroy();
    m_shaders.reset();
}

void QPointCloudRenderer::loadPLY(const QString &plyFilePath)
{
    // open stream
    std::fstream is;
    is.open(plyFilePath.toStdString().c_str(), std::fstream::in);

    // ensure format with magic header
    std::string line;
    std::getline(is, line);
    if (line != "ply") {
        throw std::runtime_error("not a ply file");
    }

    // parse header looking only for 'element vertex' section size
    m_pointsCount = 0;
    while (is.good()) {
        std::getline(is, line);
        if (line == "end_header") {
            break;
        } else {
            std::stringstream ss(line);
            std::string tag1, tag2, tag3;
            ss >> tag1 >> tag2 >> tag3;
            if (tag1 == "element" && tag2 == "vertex") {
                m_pointsCount = std::atof(tag3.c_str());
            }
        }
    }

    // read and parse 'element vertex' section
    if (m_pointsCount > 0) {
        m_pointsData.resize(m_pointsCount * POINT_STRIDE);

        std::stringstream ss;
        std::string line;
        float *p = m_pointsData.data();
        for (size_t i = 0; is.good() && i < m_pointsCount; ++i) {
            std::getline(is, line);
            ss.str(line);
            float x, y, z;
            ss >> x >> y >> z;

            *p++ = x;
            *p++ = y;
            *p++ = z;
            *p++ = i;

            // update bounds
            m_pointsBoundMax[0] = std::max(x, m_pointsBoundMax[0]);
            m_pointsBoundMax[1] = std::max(y, m_pointsBoundMax[1]);
            m_pointsBoundMax[2] = std::max(z, m_pointsBoundMax[2]);
            m_pointsBoundMin[0] = std::min(x, m_pointsBoundMin[0]);
            m_pointsBoundMin[1] = std::min(y, m_pointsBoundMin[1]);
            m_pointsBoundMin[2] = std::min(z, m_pointsBoundMin[2]);
        }

        // check if we've got exact number of points mentioned in header
        if (p - m_pointsData.data() < m_pointsData.size()) {
            throw std::runtime_error("broken ply file");
        }
    }
}

void QPointCloudRenderer::setAzimuth(float azimuth)
{
    m_azimuth = azimuth;
}

void QPointCloudRenderer::setElevation(float elevation)
{
    m_elevation = elevation;
}

void QPointCloudRenderer::setDistance(float distance)
{
    m_distance = distance;
}


