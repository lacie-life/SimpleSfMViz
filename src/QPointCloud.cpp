#include "QPointCloud.h"
#include "Constant.h"
#include <fstream>
#include <iostream>

QPointCloud::QPointCloud()
{

}

QPointCloud &QPointCloud::add_point(const QVector3D &p)
{
    auto &lower = bbox[0];
    auto &upper = bbox[1];
    if (p[0] < lower[0])
        lower[0] = p[0];
    if (p[1] < lower[1])
        lower[1] = p[1];
    if (p[2] < lower[2])
        lower[2] = p[2];
    if (p[0] > upper[0])
        upper[0] = p[0];
    if (p[1] > upper[1])
        upper[1] = p[1];
    if (p[2] > upper[2])
        upper[2] = p[2];
    points.push_back(p);
    return *this;
}

QPointCloud &QPointCloud::load_points(const QString &filename)
{
    std::ifstream file(filename.toStdString().c_str());
    if (!file.is_open())
    {
        CONSOLE << "Could not open cloud point from file " << filename;
    }

    points.clear();
    QVector3D p;
    while (!file.eof())
    {
        if (file >> p[0] >> p[1] >> p[2])
            add_point(p);
    }

    return *this;
}

QPointCloud &QPointCloud::load_colors(const QString &filename)
{
    std::ifstream file(filename.toStdString().c_str());
    if (!file.is_open())
    {
        CONSOLE << "Could not open cloud color from file " << filename;
    }

    colors.clear();
    QVector3D c;
    while (!file.eof())
    {
        if (file >> c[0] >> c[1] >> c[2])
            colors.push_back(c);
    }

    return *this;

}

QPointCloud &QPointCloud::load_normals(const QString &filename)
{
    std::ifstream file(filename.toStdString().c_str());
    if (!file.is_open())
    {
        CONSOLE << "Could not open cloud normal from file " << filename;
    }

    normals.clear();
    QVector3D n;
    while (!file.eof())
    {
        if (file >> n[0] >> n[1] >> n[2])
            normals.push_back(n);
    }

    return *this;
}
