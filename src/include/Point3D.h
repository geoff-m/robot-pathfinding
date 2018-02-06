//
// Created by Geoff M. on 11/8/17.
//
#pragma once
#ifndef PATHDRIVER_POINT3D_H
#define PATHDRIVER_POINT3D_H

#include <ostream>

class Point3D {
    bool isEmpty;

    int x, y, z;
public:
    int getX() const;
    int getY() const;
    int getZ() const;
    // Create a Point3D with the specified coordinates.
    Point3D(int x, int y, int z);

    // Create an EMPTY Point3D.
    Point3D();

    Point3D operator+(const Point3D other) const;
    Point3D operator-(const Point3D other) const;
    bool operator ==(const Point3D other) const;
    bool operator !=(const Point3D other) const;

    int manhattanNorm();

    friend std::ostream& operator <<(std::ostream& os, const Point3D& inst)
    {
        if (inst.isEmpty)
        {
            os << "[EMPTY]";
        } else {
            os << "[" << inst.x << ", " << inst.y << ", " << inst.z << "]";
        }
        return os;
    }

    friend bool operator ==(const Point3D& left, const Point3D& right)
    {
        if (left.isEmpty && right.isEmpty)
            return true;
        return left.x == right.x && left.y == right.y && left.z == right.z;
    }
};


#endif //PATHDRIVER_POINT3D_H
