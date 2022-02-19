#ifndef QPOINTCLOUD_H
#define QPOINTCLOUD_H

#include <QVector>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>
#include <QDebug>

#include <limits>
#include <tuple>
#include <vector>
#include <string>
#include <optional>

class QPointCloud
{
public:
    QPointCloud();

public:
    QVector<QVector3D> points;
    QVector<QVector3D> colors;
    QVector<QVector3D> normals;

public:
    /** add single point*/
    QPointCloud &add_point(const QVector3D &p);
    /** load point cloud from file */
    QPointCloud &load_points(const QString& filename);
    /** load point color from file */
    QPointCloud &load_colors(const QString& filename);
    /** load point normals from file */
    QPointCloud &load_normals(const QString& filename);

    inline const QVector<QVector3D> &get_points() const
    {
        return points;
    }
    inline QPointCloud &set_colors(const QVector<QVector3D> &colors_)
    {
        colors = colors_;
        return *this;
    }
    inline const QVector<QVector3D> &get_colors() const { return colors; }
    inline const QVector3D &get_bbox_min() const { return bbox[0]; }
    inline const QVector3D &get_bbox_max() const { return bbox[1]; }
    inline std::tuple<QVector3D, QVector3D> get_AABB() const { return {bbox[0], bbox[1]}; }

private:
    QVector3D bbox[2] {
        QVector3D{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()},
        QVector3D{std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min()}
    };
};

#endif // QPOINTCLOUD_H
