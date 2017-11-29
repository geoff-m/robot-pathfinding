//
// Created by Geoff M. on 11/8/17.
//

#include "include/PointD3D.h"
#include <cmath>
#include <list>

PointD3D::PointD3D(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

PointD3D PointD3D::operator+(const PointD3D other) const
{
    return PointD3D(x + other.x,
                    y + other.y,
                    z + other.z);
}

PointD3D PointD3D::operator-(const PointD3D other) const
{
    return PointD3D(x - other.x,
                    y - other.y,
                    z - other.z);
}

double PointD3D::euclideanNorm()
{
    return sqrt(x * x + y * y + z * z);
}