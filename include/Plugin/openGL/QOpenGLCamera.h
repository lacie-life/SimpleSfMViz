#ifndef QOPENGLCAMERA_H
#define QOPENGLCAMERA_H

#include <QObject>
#include <QVector3D>
#include <QVector>
#include <QMatrix>
#include <QMatrix4x4>

#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

enum CameraType {
    ORTHO,
    FREE,
};

enum CameraDirection {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD,
};

class QOpenGLCamera : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QVector3D position READ position WRITE setPosition NOTIFY positionChanged)
    Q_PROPERTY(QMatrix4x4 mvp READ mvp WRITE setMVP NOTIFY mvpChanged)

public:
    explicit QOpenGLCamera(QObject *parent = nullptr);

    ~QOpenGLCamera();

    QVector3D position() const;
    QMatrix4x4 mvp() const;

    void reset();
    //This function updates the camera
    //Depending on the current camera mode, the projection and viewport matricies are computed
    //Then the position and location of the camera is updated
    void update();

    //Given a specific moving direction, the camera will be moved in the appropriate direction
    //For a spherical camera this will be around the look_at point
    //For a free camera a delta will be computed for the direction of movement.
    void move(CameraDirection dir);
    //Change the pitch (up, down) for the free camera
    void changePitch(float degrees);
    //Change heading (left, right) for the free camera
    void changeHeading(float degrees);

    //Change the heading and pitch of the camera based on the 2d movement of the mouse
    void move2D(int x, int y);

    //Setting Functions
    //Changes the camera mode, only three valid modes, Ortho, Free, and Spherical
    void setMode(CameraType cam_mode);

    //Set's the look at point for the camera
    void setLookAt(QVector3D pos);
    //Changes the Field of View (FOV) for the camera
    void setFOV(double fov);
    //Change the viewport location and size
    void setViewport(int loc_x, int loc_y, int width, int height);
    //Change the clipping distance for the camera
    void setClipping(double near_clip_distance, double far_clip_distance);

    void setDistance(double cam_dist);
    void setPos(int button, int state, int x, int y);

    //Getting Functions
    CameraType getMode();
    void getViewport(int &loc_x, int &loc_y, int &width, int &height);
    void getMatricies(glm::mat4 &P, glm::mat4 &V, glm::mat4 &M);

public:
    CameraType camera_mode;

    int viewport_x;
    int viewport_y;

    int window_width;
    int window_height;

    double aspect;
    double field_of_view;
    double near_clip;
    double far_clip;

    float camera_scale;
    float camera_heading;
    float camera_pitch;

    float max_pitch_rate;
    float max_heading_rate;
    bool move_camera;

    glm::vec3 camera_position;
    glm::vec3 camera_position_delta;
    glm::vec3 camera_look_at;
    glm::vec3 camera_direction;

    glm::vec3 camera_up;
    glm::vec3 mouse_position;

    glm::mat4 projection;
    glm::mat4 view;
    glm::mat4 model;
    glm::mat4 MVP;

public:
    static QVector3D glmVec3toQVector3 (glm::vec3 vec);
    static glm::vec3 QVector3toglmVec3 (QVector3D vec);
    static QMatrix4x4 glmMat4toQMatrix4 (glm::mat4 mat);
    static glm::mat4 QMatrix4toglmMat4 (QMatrix4x4 mat);

public slots:
    void setPosition(QVector3D position);
    void setMVP (QMatrix4x4 mvp);

signals:
    void positionChanged(QVector3D position);
    void mvpChanged(QMatrix4x4 mvp);

};

#endif // QOPENGLCAMERA_H
