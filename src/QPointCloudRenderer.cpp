#include "QPointCloudRenderer.h"
#include "QCameraControl.h"
#include "Constant.h"


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

    glClearColor(0, 0, 0, 1.0);

    loadPLY(plyFilePath);

    CONSOLE << "Initialize";

    // the world is still for now
    m_worldMatrix.setToIdentity();

    CONSOLE << "Fucking VAO";

    if (m_vao->isCreated())
        return; // already initialized
    }

    if (!m_vao->create())
        qFatal("Unable to create VAO");

    m_vao->bind();

    //
    // create shaders and map attributes
    //
    m_shaders.reset(new QOpenGLShaderProgram());

    auto vsLoaded = m_shaders->addShaderFromSourceFile(QOpenGLShader::Vertex, "/home/jun/Github/GreenHouseAR/assest/vertex_shader.glsl");
    auto fsLoaded = m_shaders->addShaderFromSourceFile(QOpenGLShader::Fragment, "/home/jun/Github/GreenHouseAR/assest/fragment_shader.glsl");


    m_coordinateMirroring = cm;

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

    m_vertexBuffer->create();
    m_vertexBuffer->bind();
    m_vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_vertexBuffer->allocate(m_pointsData.constData(), m_pointsData.size() * sizeof(GLfloat));

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    f->glEnableVertexAttribArray(0);
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), 0);
    f->glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat) + sizeof(GLfloat), reinterpret_cast<void *>(3*sizeof(GLfloat)));

    m_vao->release();
    // m_vertexBuffer->release();

}

void QPointCloudRenderer::render()
{

    CONSOLE << "Render";

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glClearColor(0.0, 0.0, 0.0, 1.0);
    // ensure GL flags
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glEnable(GL_DEPTH_TEST);
    f->glEnable(GL_VERTEX_PROGRAM_POINT_SIZE); //required for gl_PointSize

    // position and angles
    m_cameraMatrix.setToIdentity();
    m_cameraMatrix.translate(m_position.x(), m_position.y(), m_position.z());
    m_cameraMatrix.rotate(m_xRotation, 1, 0, 0);
    m_cameraMatrix.rotate(m_yRotation, 0, 1, 0);
    m_cameraMatrix.rotate(m_zRotation, 0, 0, 1);

    qDebug() << "Position: " << m_position.x() << " " << m_position.y()<< " " << m_position.z();
    qDebug() << "Rotation: " << m_xRotation << " " << m_yRotation << " " << m_zRotation;

    m_projectionMatrix.setToIdentity();
    GLint viewportSize[4];
    f->glGetIntegerv(GL_VIEWPORT, viewportSize);
    m_projectionMatrix.perspective(30, float(viewportSize[2]) / viewportSize[3], 0.01, 1000);

    // set clipping planes
    f->glEnable(GL_CLIP_PLANE1);
    f->glEnable(GL_CLIP_PLANE2);
    const double rearClippingPlane[] = {0., 0., -1., m_rearClippingDistance};
    glClipPlane(GL_CLIP_PLANE1 , rearClippingPlane);
    const double frontClippingPlane[] = {0., 0., 1., m_frontClippingPlaneDistance};
    glClipPlane(GL_CLIP_PLANE2 , frontClippingPlane);

    //
    // draw points cloud
    //
    const auto viewMatrix = m_projectionMatrix * m_cameraMatrix * m_worldMatrix;

    CONSOLE << "View Matrix: " << viewMatrix;

    QMatrix4x4 viewMatrix2(0.624116,         0,  0.743792,         0,
                           0,   1.42815,         0, -0.142815,
                           0.766198,         0, -0.642916,  0.180038,
                           0.766044,         0, -0.642788,       0.2 );

    m_shaders->bind();
    m_shaders->setUniformValue("pointsCount", static_cast<GLfloat>(m_pointsCount));
    m_shaders->setUniformValue("viewMatrix", viewMatrix);
    m_shaders->setUniformValue("pointSize", m_pointSize);
    m_shaders->setUniformValue("colorAxisMode", static_cast<GLfloat>(m_colorMode));
    m_shaders->setUniformValue("pointsBoundMin", m_pointsBoundMin);
    m_shaders->setUniformValue("pointsBoundMax", m_pointsBoundMax);

    m_vao->bind();
    f->glDrawArrays(GL_POINTS, 0, m_pointsData.size());
    m_shaders->release();
    m_vao->release();

    CONSOLE << "Point: " << m_pointsCount << " " << m_pointsData.size();

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

void QPointCloudRenderer::setFrontClippingPlaneDistance(double distance) {
    m_frontClippingPlaneDistance = distance;
}


void QPointCloudRenderer::setRearClippingDistance(double distance) {
    m_rearClippingDistance = distance;
}


void QPointCloudRenderer::setPosition(QVector3D position) {
    m_position = position;
}


void QPointCloudRenderer::setxRotation(int angle)
{ 
    m_xRotation = angle;
}


void QPointCloudRenderer::setyRotation(int angle)
{ 
    m_yRotation = angle;
}

void QPointCloudRenderer::setzRotation(int angle)
{
    m_zRotation = angle;
}

