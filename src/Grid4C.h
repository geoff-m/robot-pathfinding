//
// Created by Geoff M. on 11/8/17.
//
#pragma once
#include <stdlib.h>
#include "Point3D.h"
#include "PointD3D.h"
#ifndef PATHDRIVER_GRID_H
#define PATHDRIVER_GRID_H
#include <list>

class Grid4C
{
private:
    int rows, columns, levels;
    float rowSpacing, columnSpacing, levelSpacing;
    float originX, originY, originZ;
    //int*** data; // 3D array showing who--if anyone--occupies each location.

public:
    Grid4C(const PointD3D worldOrigin,
           int rows, int columns, int levels,
           float rowSpacing, float columnSpacing, float levelSpacing);

    double getDistance(const Point3D x, const Point3D y) const;

    Point3D getGridPoint(const PointD3D worldPoint) const;
    PointD3D getWorldPoint(const Point3D gridPoint) const;
    std::list<PointD3D> getWorldPoints(const std::list<Point3D> gridPoints) const;

    std::list<Point3D> getNeighbors(const Point3D gridPoint) const;

    bool contains(const Point3D point) const;

    int getRowCount() const;
    int getColumnCount() const;
    int getLevelCount() const;



    ~Grid4C();
};

#endif //PATHDRIVER_GRID_H
