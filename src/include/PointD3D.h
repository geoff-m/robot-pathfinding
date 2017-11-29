//
// Created by Geoff M. on 11/8/17.
//
#pragma once
#ifndef PATHDRIVER_POINTD3D_H
#define PATHDRIVER_POINTD3D_H

#include <string>
#include <ostream>

struct PointD3D {
    double x, y, z;
    PointD3D(double x, double y, double z);

    PointD3D operator+(const PointD3D other) const;
    PointD3D operator-(const PointD3D other) const;

    double euclideanNorm();

    friend std::ostream& operator<<(std::ostream& os, const PointD3D& inst)
    {
        os << "[" << inst.x << ", " << inst.y << ", " << inst.z << "]";
        return os;
    }
};


#endif //PATHDRIVER_POINTD3D_H
