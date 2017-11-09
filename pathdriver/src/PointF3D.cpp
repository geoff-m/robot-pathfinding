//
// Created by Geoff M. on 11/8/17.
//

#include "include/PointF3D.h"
#include <cmath>

PointF3D::PointF3D(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

PointF3D PointF3D::operator+(const PointF3D other) const
{
    return PointF3D(x + other.x,
                    y + other.y,
                    z + other.z);
}

PointF3D PointF3D::operator-(const PointF3D other) const
{
    return PointF3D(x - other.x,
                    y - other.y,
                    z - other.z);
}

float PointF3D::euclideanNorm()
{
    return sqrtf(x * x + y * y + z * z);
}