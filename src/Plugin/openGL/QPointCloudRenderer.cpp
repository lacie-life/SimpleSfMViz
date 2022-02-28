#include "openGL/QPointCloudRenderer.h"
#include "openGL/QCameraControl.h"
#include "AppConstant.h"

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
    , m_positionsBuffer(new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer))
    , m_colorsBuffer(new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer))
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

void QPointCloudRenderer::initialize(const QString &filePath)
{
    glClearColor(0, 0, 0, 1.0);

    // loadPLY(plyFilePath);

    m_pointCloud = new QPointCloud(this);

    m_pointCloud->loadPointCloud(filePath);

    CONSOLE << "Initialize";

    CONSOLE << "Fucking VAO";

    if (m_vao->isCreated())
    {
        CONSOLE << "VAO Failed";
        return; // already initialized
    }

    if (!m_vao->create())
        qFatal("Unable to create VAO");

    m_vao->bind();

    //
    // create shaders and map attributes
    //
    m_shaders.reset(new QOpenGLShaderProgram());

    auto vsLoaded = m_shaders->addShaderFromSourceFile(QOpenGLShader::Vertex, "/home/jun/Github/GreenHouseAR/assest/shader/vertex_shader.glsl");
    auto fsLoaded = m_shaders->addShaderFromSourceFile(QOpenGLShader::Fragment, "/home/jun/Github/GreenHouseAR/assest/shader/fragment_shader.glsl");

    CONSOLE << "Shader Program Initialized";

    assert(vsLoaded && fsLoaded);

    // vector attributes
//    m_shaders->bindAttributeLocation("vertex", 0);
//    m_shaders->bindAttributeLocation("pointRowIndex", 1);
//    m_shaders->bindAttributeLocation("color", 2);

    // constants
//      m_shaders->bind();
//    m_shaders->setUniformValue("lightPos", QVector3D(0, 0, 50));
//    m_shaders->setUniformValue("pointsCount", static_cast<GLfloat>(m_pointsCount));

    // PLY file reader
//    m_vertexBuffer->create();
//    m_vertexBuffer->bind();
//    m_vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
//    m_vertexBuffer->allocate(m_pointsData.constData(), m_pointsData.size() * sizeof(GLfloat));


    // PCLPointCloud2
    const QVector<QVector3D> vertices = m_pointCloud->vertices();
    CONSOLE << "Number vertice: " << vertices.size();
    if (!m_positionsBuffer->create())
        qFatal("Unable to create position buffer");
    m_positionsBuffer->bind();
    m_positionsBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    CONSOLE << "?";
    m_positionsBuffer->allocate(vertices.constData(), vertices.size() * sizeof(QVector3D));

    const QVector<QVector3D> colors = m_pointCloud->colors();
    CONSOLE << "Number colors: " << colors.size();
    if (!m_colorsBuffer->create())
        qFatal("Unable to create color buffer");
    m_colorsBuffer->bind();
    m_colorsBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_colorsBuffer->allocate(colors.constData(), colors.size() * sizeof(QVector3D));

//    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
//    f->glEnableVertexAttribArray(0);
//    f->glEnableVertexAttribArray(1);
//    f->glEnableVertexAttribArray(2);
//    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), 0);
//    f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), 0);
//    f->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), reinterpret_cast<void *>(3*sizeof(GLfloat)));

    m_shaders->bind();

    m_positionsBuffer->bind();
    m_shaders->enableAttributeArray("vertexPosition");
    m_shaders->setAttributeBuffer("vertexPosition", GL_FLOAT, 0, 3);

    m_colorsBuffer->bind();
    m_shaders->enableAttributeArray("vertexColor");
    m_shaders->setAttributeBuffer("vertexColor", GL_FLOAT, 0, 3);

    m_shaders->link();
    m_shaders->release();

    m_vao->release();
    // m_vertexBuffer->release();

    CONSOLE << "Initialized";
}

void QPointCloudRenderer::render()
{
    CONSOLE << "Render";

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glClearColor(1.0, 1.0, 1.0, 1.0);
    // ensure GL flags
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glEnable(GL_DEPTH_TEST);
    f->glEnable(GL_VERTEX_PROGRAM_POINT_SIZE); //required for gl_PointSize

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
    f->glGetIntegerv(GL_VIEWPORT, viewportSize);

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

    //
    // draw points cloud
    //
    const auto mvp = modelViewProjectionMatrix;

    CONSOLE << "MVP Matrix: " << mvp;

    m_shaders->bind();
    m_shaders->setUniformValue("viewMatrix", mvp);
    m_shaders->setUniformValue("pointSize", m_pointSize);

    m_vao->bind();
    f->glDrawArrays(GL_POINTS, 0, m_pointCloud->pointsNumber());
    m_shaders->release();
    m_vao->release();

    CONSOLE << "Point: " << m_pointCloud->pointsNumber();
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






