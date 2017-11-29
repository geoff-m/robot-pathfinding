//
// Created by Geoff M. on 11/8/17.
//
#pragma once
#ifndef PATHDRIVER_POINT3D_H
#define PATHDRIVER_POINT3D_H

#include <ostream>

struct Point3D {
    int x, y, z;
    Point3D(int x, int y, int z);

    Point3D operator+(const Point3D other) const;
    Point3D operator-(const Point3D other) const;

    int manhattanNorm();

    friend std::ostream& operator<<(std::ostream& os, const Point3D& inst)
    {
        os << "[" << inst.x << ", " << inst.y << ", " << inst.z << "]";
        return os;
    }
};


#endif //PATHDRIVER_POINT3D_H
