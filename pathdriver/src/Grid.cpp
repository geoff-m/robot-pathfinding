//
// Created by student on 11/8/17.
//

#include "include/Grid.h"
#include <stdexcept>

int clamp(int min, int val, int max);

// Represents a 6-connected 3-dimensional grid.
Grid4C::Grid4C(const PointF3D worldOrigin,
               int rows, int columns, int levels,
               float rowSpacing, float columnSpacing, float levelSpacing)
{
    this->originX = worldOrigin.x;
    this->originY = worldOrigin.y;
    this->originZ = worldOrigin.z;
    this->rows = rows;
    this->columns = columns;
    this->levels = levels;
    this->rowSpacing = rowSpacing;
    this->columnSpacing = columnSpacing;
    this->levelSpacing = levelSpacing;

}

double Grid4C::getDistance(const Point3D x, const Point3D y) const
{
    // Compute the Manhattan distance, since this a 4-/6-connected grid.
    return (x-y).manhattanNorm();
}

// Returns the grid point that is nearest to the given world point.
Point3D Grid4C::getGridPoint(const PointF3D worldPoint) const
{
    int row = (int) (0.5 + (worldPoint.x - originX) / rowSpacing);
    int col = (int) (0.5 + (worldPoint.y - originY) / columnSpacing);
    int lvl = (int) (0.5 + (worldPoint.y - originZ) / levelSpacing);

    row = clamp(0, row, rows - 1);
    col = clamp(0, col, columns - 1);
    lvl = clamp(0, lvl, levels - 1);

    return Point3D(row, col, lvl);
}

PointF3D Grid4C::getWorldPoint(const Point3D gridPoint) const
{
    if (!contains(gridPoint))
    {
        throw std::invalid_argument("That point is not in the grid!");
    }
    return PointF3D(originX + gridPoint.x * rowSpacing,
                    originY + gridPoint.y * columnSpacing,
                    originZ + gridPoint.z * levelSpacing);
}

bool Grid4C::contains(Point3D point) const
{
    return point.x >= rows && point.x < 0 &&
            point.y >= columns && point.y < 0 &&
            point.z >= levels && point.z < 0;
}

int clamp(int min, int val, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
    return val;
}