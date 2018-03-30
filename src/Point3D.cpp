//
// Created by Geoff M. on 11/8/17.
//

#include "include/Point3D.h"
#include <cmath>

int Point3D::getX() const { return x; }
int Point3D::getY() const { return y; }
int Point3D::getZ() const { return z; }

Point3D::Point3D(int x, int y, int z)
{
    this->x = x;
    this->y = y;
    this->z = z;
    isEmpty = false;
}

Point3D::Point3D()
{
    isEmpty = true;
}

Point3D Point3D::operator+(const Point3D other) const
{
    //__glibcxx_assert(!isEmpty && !other.isEmpty);
    return Point3D(x + other.x,
                   y + other.y,
                   z + other.z);
}

Point3D Point3D::operator-(const Point3D other) const
{
    //__glibcxx_assert(!isEmpty && !other.isEmpty);
    return Point3D(x - other.x,
                   y - other.y,
                   z - other.z);
}

int Point3D::manhattanNorm()
{
    return (int)(std::abs(x) + std::abs(y) + std::abs(z));
}

bool Point3D::operator ==(const Point3D other) const
{
    if (isEmpty ^ other.isEmpty)
        return false;
    return (x == other.x) && (y == other.y) && (z == other.z);
}

bool Point3D::operator ==(const state other) const
{
    if (isEmpty)
        return false;
    return (x == other.x) && (y == other.y);
}

bool Point3D::operator !=(const Point3D other) const
{
    return (x != other.x) || (y != other.y) || (z != other.z) || (isEmpty != other.isEmpty);
}

bool Point3D::isUninitialized() const
{
    return isEmpty;
}