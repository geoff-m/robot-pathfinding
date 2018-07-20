//
// Created by Geoff M. on 11/8/17.
//

#include <stdlib.h>
#include "Point3D.h"
#include "PointF3D.h"
#ifndef PATHDRIVER_GRID_H
#define PATHDRIVER_GRID_H

class Grid4C
{
private:
    int rows, columns, levels;
    float rowSpacing, columnSpacing, levelSpacing;
    float originX, originY, originZ;

public:
    Grid4C(const PointF3D worldOrigin,
           int rows, int columns, int levels,
           float rowSpacing, float columnSpacing, float levelSpacing);

    double getDistance(const Point3D x, const Point3D y) const;

    Point3D getGridPoint(const PointF3D worldPoint) const;
    PointF3D getWorldPoint(const Point3D gridPoint) const;

    bool contains(Point3D point) const;
};

#endif //PATHDRIVER_GRID_H
