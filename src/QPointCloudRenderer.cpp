#include "QPointCloudRenderer.h"

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
    , m_vao()
    , m_vertexBuffer(new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer))
    , m_shaders()
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

void QPointCloudRenderer::initialize(const QString &plyFilePath)
{
    loadPLY(plyFilePath);

    if (m_vao->isCreated())
        return; // already initialized

    //
    // create shaders and map attributes
    //
    m_shaders.reset(new QOpenGLShaderProgram());
    auto vsLoaded = m_shaders->addShaderFromSourceFile(QOpenGLShader::Vertex, "qrc:/shader/assest/vertex_shader.glsl");
    auto fsLoaded = m_shaders->addShaderFromSourceFile(QOpenGLShader::Fragment, "qrc:/shader/assest/fragment_shader.glsl");
    assert(vsLoaded && fsLoaded);
    // vector attributes
    m_shaders->bindAttributeLocation("vertex", 0);
    m_shaders->bindAttributeLocation("pointRowIndex", 1);
    // constants
    m_shaders->bind();
    m_shaders->setUniformValue("lightPos", QVector3D(0, 0, 50));
    m_shaders->setUniformValue("pointsCount", static_cast<GLfloat>(m_pointsCount));
    m_shaders->link();
    m_shaders->release();

    if (!m_vao->create())
            qFatal("Unable to create VAO");

    m_vao->bind();

    m_vertexBuffer->create();
    m_vertexBuffer->bind();
    m_vertexBuffer->allocate(m_pointsData.constData(), m_pointsData.size() * sizeof(GLfloat));


    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), reinterpret_cast<void *>(3*sizeof(GLfloat)));
    m_vertexBuffer->release();
}

void QPointCloudRenderer::render()
{
    // ensure GL flags
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      glEnable(GL_DEPTH_TEST);
      glEnable(GL_VERTEX_PROGRAM_POINT_SIZE); //required for gl_PointSize

      //
      // set camera
      //
      const QCameraState camera = m_currentCamera->currentState();
      // position and angles
      m_cameraMatrix.setToIdentity();
      m_cameraMatrix.translate(camera.position.x(), camera.position.y(), camera.position.z());
      m_cameraMatrix.rotate(camera.rotation.x(), 1, 0, 0);
      m_cameraMatrix.rotate(camera.rotation.y(), 0, 1, 0);
      m_cameraMatrix.rotate(camera.rotation.z(), 0, 0, 1);

      // set clipping planes
      glEnable(GL_CLIP_PLANE1);
      glEnable(GL_CLIP_PLANE2);
      const double rearClippingPlane[] = {0., 0., -1., camera.rearClippingDistance};
      glClipPlane(GL_CLIP_PLANE1 , rearClippingPlane);
      const double frontClippingPlane[] = {0., 0., 1., camera.frontClippingDistance};
      glClipPlane(GL_CLIP_PLANE2 , frontClippingPlane);

      //
      // draw points cloud
      //
      m_vao->bind();
      const auto viewMatrix = m_projectionMatrix * m_cameraMatrix * m_worldMatrix;
      m_shaders->bind();
      m_shaders->setUniformValue("pointsCount", static_cast<GLfloat>(m_pointsCount));
      m_shaders->setUniformValue("viewMatrix", viewMatrix);
      m_shaders->setUniformValue("pointSize", m_pointSize);
      m_shaders->setUniformValue("colorAxisMode", static_cast<GLfloat>(m_colorMode));
      m_shaders->setUniformValue("pointsBoundMin", m_pointsBoundMin);
      m_shaders->setUniformValue("pointsBoundMax", m_pointsBoundMax);
      glDrawArrays(GL_POINTS, 0, m_pointsData.size());
      m_shaders->release();

      drawFrameAxis();
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

void QPointCloudRenderer::drawFrameAxis()
{
    glBegin(GL_LINES);
    QMatrix4x4 mvMatrix = m_cameraMatrix * m_worldMatrix;
    mvMatrix.scale(0.05); // make it small
    for (auto vertex : m_axesLines) {
        const auto translated = m_projectionMatrix * mvMatrix * vertex.first;
        glColor3f(vertex.second.red(), vertex.second.green(), vertex.second.blue());
        glVertex3f(translated.x(), translated.y(), translated.z());
    }
    glEnd();
}

void QPointCloudRenderer::setCameraState(const QCameraState &state)
{
    m_currentCamera->setCurrentState(state);

}






