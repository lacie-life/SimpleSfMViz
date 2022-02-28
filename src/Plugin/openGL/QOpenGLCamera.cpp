#include "openGL/QOpenGLCamera.h"
#include "AppConstant.h"

QOpenGLCamera::QOpenGLCamera(QObject *parent)
    : QObject{parent}
    , camera_mode(FREE)
    , field_of_view(45)
    , camera_scale(0.5f)
    , max_pitch_rate(5)
    , max_heading_rate(5)
    , move_camera(false)
    , camera_position_delta(glm::vec3(0, 0, 0))
    , camera_up(glm::vec3(0, 1, 0))
{

}

QOpenGLCamera::~QOpenGLCamera()
{

}

QVector3D QOpenGLCamera::position() const
{
    return glmVec3toQVector3(camera_position);
}

QMatrix4x4 QOpenGLCamera::mvp() const
{
    return glmMat4toQMatrix4(MVP);
}

void QOpenGLCamera::reset()
{
    camera_up = glm::vec3(0, 1, 0);
}

void QOpenGLCamera::update()
{
    camera_direction = glm::normalize(camera_look_at - camera_position);
    //need to set the matrix state. this is only important because lighting doesn't work if this isn't done
    glViewport(viewport_x, viewport_y, window_width, window_height);

    if (camera_mode == ORTHO) {
        //our projection matrix will be an orthogonal one in this case
        //if the values are not floating point, this command does not work properly
        //need to multiply by aspect!!! (otherise will not scale properly)
        projection = glm::ortho(-1.5f * float(aspect), 1.5f * float(aspect), -1.5f, 1.5f, -10.0f, 10.f);
    } else if (camera_mode == FREE) {
        projection = glm::perspective(field_of_view, aspect, near_clip, far_clip);
        //detmine axis for pitch rotation
        glm::vec3 axis = glm::cross(camera_direction, camera_up);
        //compute quaternion for pitch based on the camera pitch angle
        glm::quat pitch_quat = glm::angleAxis(camera_pitch, axis);
        //determine heading quaternion from the camera up vector and the heading angle
        glm::quat heading_quat = glm::angleAxis(camera_heading, camera_up);
        //add the two quaternions
        glm::quat temp = glm::cross(pitch_quat, heading_quat);
        temp = glm::normalize(temp);
        //update the direction from the quaternion
        camera_direction = glm::rotate(temp, camera_direction);
        //add the camera delta
        camera_position += camera_position_delta;
        //set the look at to be infront of the camera
        camera_look_at = camera_position + camera_direction * 1.0f;
        //damping for smooth camera
        camera_heading *= .5;
        camera_pitch *= .5;
        camera_position_delta = camera_position_delta * .8f;
    }
    //compute the MVP
    view = glm::lookAt(camera_position, camera_look_at, camera_up);
    model = glm::mat4(1.0f);
    MVP = projection * view * model;

    setMVP(glmMat4toQMatrix4(MVP));
}

//Setting Functions
void QOpenGLCamera::setMode(CameraType cam_mode) {
    camera_mode = cam_mode;
    camera_up = glm::vec3(0, 1, 0);
}

void QOpenGLCamera::setPosition(QVector3D pos) {
    camera_position = QVector3toglmVec3(pos);

    emit positionChanged(pos);
}

void QOpenGLCamera::setMVP(QMatrix4x4 mvp)
{
    MVP = QMatrix4toglmMat4(mvp);

    emit mvpChanged(mvp);
}

void QOpenGLCamera::setLookAt(QVector3D pos) {
    camera_look_at = QVector3toglmVec3(pos);
}
void QOpenGLCamera::setFOV(double fov) {
    field_of_view = fov;
}
void QOpenGLCamera::setViewport(int loc_x, int loc_y, int width, int height) {
    viewport_x = loc_x;
    viewport_y = loc_y;
    window_width = width;
    window_height = height;
    //need to use doubles division here, it will not work otherwise and it is possible to get a zero aspect ratio with integer rounding
    aspect = double(width) / double(height);
    ;
}
void QOpenGLCamera::setClipping(double near_clip_distance, double far_clip_distance) {
    near_clip = near_clip_distance;
    far_clip = far_clip_distance;
}

void QOpenGLCamera::move(CameraDirection dir) {
    if (camera_mode == FREE) {
        switch (dir) {
            case UP:
                CONSOLE << "UPPPPPPPPPP";
                camera_position_delta += camera_up * camera_scale;
                break;
            case DOWN:
                CONSOLE << "DOWNNNNNNNN";
                camera_position_delta -= camera_up * camera_scale;
                break;
            case LEFT:
                CONSOLE << "LEFTTTTTTTTTTTTT";
                camera_position_delta -= glm::cross(camera_direction, camera_up) * camera_scale;
                break;
            case RIGHT:
                CONSOLE << "RIGHTTTTTT";
                camera_position_delta += glm::cross(camera_direction, camera_up) * camera_scale;
                break;
            case FORWARD:
                CONSOLE << "FORWARDDDDDD";
                camera_position_delta += camera_direction * camera_scale;
                break;
            case BACKWARD:
                CONSOLE << "BACKWARDDDD";
                camera_position_delta -= camera_direction * camera_scale;
                break;
        }
    }
    update();
}
void QOpenGLCamera::changePitch(float degrees) {
    //Check bounds with the max pitch rate so that we aren't moving too fast
    if (degrees < -max_pitch_rate) {
        degrees = -max_pitch_rate;
    } else if (degrees > max_pitch_rate) {
        degrees = max_pitch_rate;
    }
    camera_pitch += degrees;

    //Check bounds for the camera pitch
    if (camera_pitch > 360.0f) {
        camera_pitch -= 360.0f;
    } else if (camera_pitch < -360.0f) {
        camera_pitch += 360.0f;
    }
}
void QOpenGLCamera::changeHeading(float degrees) {
    //Check bounds with the max heading rate so that we aren't moving too fast
    if (degrees < -max_heading_rate) {
        degrees = -max_heading_rate;
    } else if (degrees > max_heading_rate) {
        degrees = max_heading_rate;
    }
    //This controls how the heading is changed if the camera is pointed straight up or down
    //The heading delta direction changes
    if (camera_pitch > 90 && camera_pitch < 270 || (camera_pitch < -90 && camera_pitch > -270)) {
        camera_heading -= degrees;
    } else {
        camera_heading += degrees;
    }
    //Check bounds for the camera heading
    if (camera_heading > 360.0f) {
        camera_heading -= 360.0f;
    } else if (camera_heading < -360.0f) {
        camera_heading += 360.0f;
    }
}
void QOpenGLCamera::move2D(int x, int y) {
    //compute the mouse delta from the previous mouse position
    glm::vec3 mouse_delta(x, y, 0);

    CONSOLE << "MOVING";
    changeHeading(.08f * mouse_delta.x);
    changePitch(.08f * mouse_delta.y);

    update();
}

void QOpenGLCamera::setPos(int button, int state, int x, int y) {
    if (button == 3 && state == GLUT_DOWN) {
        camera_position_delta += camera_up * .05f;
    } else if (button == 4 && state == GLUT_DOWN) {
        camera_position_delta -= camera_up * .05f;
    } else if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        move_camera = true;
    } else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        move_camera = false;
    }
    mouse_position = glm::vec3(x, y, 0);
}

CameraType QOpenGLCamera::getMode() {
    return camera_mode;
}

void QOpenGLCamera::getViewport(int &loc_x, int &loc_y, int &width, int &height) {
    loc_x = viewport_x;
    loc_y = viewport_y;
    width = window_width;
    height = window_height;
}

void QOpenGLCamera::getMatricies(glm::mat4 &P, glm::mat4 &V, glm::mat4 &M) {
    P = projection;
    V = view;
    M = model;
}

QVector3D QOpenGLCamera::glmVec3toQVector3(glm::vec3 vec)
{
    return QVector3D(vec[0], vec[1], vec[2]);
}

glm::vec3 QOpenGLCamera::QVector3toglmVec3(QVector3D vec)
{
    return glm::vec3(vec[0], vec[1], vec[2]);
}

QMatrix4x4 QOpenGLCamera::glmMat4toQMatrix4(glm::mat4 mat)
{
    return QMatrix4x4(mat[0][0], mat[0][1], mat[0][2], mat[0][3],
                      mat[1][0], mat[1][1], mat[1][2], mat[1][3],
                      mat[2][0], mat[2][1], mat[2][2], mat[2][3],
                      mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
}

glm::mat4 QOpenGLCamera::QMatrix4toglmMat4(QMatrix4x4 mat)
{
    QMatrix4x4 _mat = mat.transposed();
    return glm::mat4(_mat(0, 0), _mat(0, 1), _mat(0, 2), _mat(0, 3),
                     _mat(1, 0), _mat(1, 1), _mat(1, 2), _mat(1, 3),
                     _mat(2, 0), _mat(2, 1), _mat(2, 2), _mat(2, 3),
                     _mat(3, 0), _mat(3, 1), _mat(3, 2), _mat(3, 3));
}


