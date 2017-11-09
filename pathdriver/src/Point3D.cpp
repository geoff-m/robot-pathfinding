//
// Created by Geoff M. on 11/8/17.
//

#include "include/Point3D.h"
#include <cmath>

Point3D::Point3D(int x, int y, int z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Point3D Point3D::operator+(const Point3D other) const
{
    return Point3D(x + other.x,
                   y + other.y,
                   z + other.z);
}

Point3D Point3D::operator-(const Point3D other) const
{
    return Point3D(x - other.x,
                   y - other.y,
                   z - other.z);
}

int Point3D::manhattanNorm()
{
    return std::abs(x) + std::abs(y) + std::abs(z);
}