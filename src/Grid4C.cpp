//
// Created by student on 11/8/17.
//

#include "Grid4C.h"
#include <stdexcept>
#include <list>
#include <cmath>

int clamp(int min, int val, int max);

// Represents a 6-connected 3-dimensional grid.
Grid4C::Grid4C(const PointD3D worldOrigin,
               int rows, int columns, int levels,
               float rowSpacing, float columnSpacing, float levelSpacing)
{
    this->originX = worldOrigin.getX();
    this->originY = worldOrigin.getY();
    this->originZ = worldOrigin.getZ();
    this->rows = rows;
    this->columns = columns;
    this->levels = levels;
    this->rowSpacing = rowSpacing;
    this->columnSpacing = columnSpacing;
    this->levelSpacing = levelSpacing;

    // Initialize data array.
    /*data = new int** [rows]();
    for (int r = 0; r < rows; ++r)
    {
        data[r] = new int* [columns]();
        for (int c = 0; c < columns; ++c)
        {
            data[r][c] = new int [levels];
            for (int l = 0; l < levels; ++l)
            {
                data[r][c][l] = EMPTY_CELL;
            }
        }
    }*/
}

Grid4C::~Grid4C()
{
    // Deallocate cell array.
    /*for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < columns; ++c)
            delete[] data[r][c];
        delete[] data[r];
    }
    delete[] data;*/
}

double Grid4C::getDistance(const Point3D x, const Point3D y) const
{
    // Compute the Manhattan distance, since this a 4-/6-connected grid.
    return (x-y).manhattanNorm();
}

// Returns the grid point that is nearest to the given world point.
Point3D Grid4C::getGridPoint(const PointD3D worldPoint) const
{
    int row = (int) lround((worldPoint.getX() - originX) / rowSpacing);
    int col = (int) lround((worldPoint.getY() - originY) / columnSpacing);
    int lvl = (int) lround((worldPoint.getZ() - originZ) / levelSpacing);

    row = clamp(0, row, rows - 1);
    col = clamp(0, col, columns - 1);
    lvl = clamp(0, lvl, levels - 1);

    return {row, col, lvl};
}

PointD3D Grid4C::getWorldPoint(const Point3D gridPoint) const
{
    if (!contains(gridPoint))
    {
        throw std::invalid_argument("That point is not in the grid!");
    }
    return {originX + gridPoint.getX() * rowSpacing,
            originY + gridPoint.getY() * columnSpacing,
            originZ + gridPoint.getZ() * levelSpacing};
}

std::list<PointD3D> Grid4C::getWorldPoints(const std::list<Point3D> gridPoints) const {
    std::list<PointD3D> ret;
    for (auto iter = gridPoints.begin();
         iter != gridPoints.end();
         ++iter)
    {
        ret.push_back(this->getWorldPoint(*iter));
    }
    return ret;
}

std::list<Point3D> Grid4C::getNeighbors(const Point3D gridPoint) const
{
    bool top = gridPoint.getZ() == levels;
    bool bottom = gridPoint.getZ() == 0;
    bool front = gridPoint.getX() == 0;
    bool back = gridPoint.getX() == rows;
    bool left = gridPoint.getY() == 0;
    bool right = gridPoint.getY() == columns;
    std::list<Point3D> ret;
    if (!top)
    {
        ret.emplace_back(gridPoint.getX(), gridPoint.getY(), gridPoint.getZ() + 1);
    }
    if (!bottom)
    {
        ret.emplace_back(gridPoint.getX(), gridPoint.getY(), gridPoint.getZ() - 1);
    }
    if (!front)
    {
        ret.emplace_back(gridPoint.getX() - 1, gridPoint.getY(), gridPoint.getZ());
    }
    if (!back)
    {
        ret.emplace_back(gridPoint.getX() + 1, gridPoint.getY(), gridPoint.getZ());
    }
    if (!left)
    {
        ret.emplace_back(gridPoint.getX(), gridPoint.getY() - 1, gridPoint.getZ());
    }
    if (!right)
    {
        ret.emplace_back(gridPoint.getX(), gridPoint.getY() + 1, gridPoint.getZ());
    }
    return ret;
}

bool Grid4C::contains(Point3D point) const
{
    return point.getX() <= rows && point.getX() >= 0 &&
            point.getY() <= columns && point.getY() >= 0 &&
            point.getZ() <= levels && point.getZ() >= 0;
}

int Grid4C::getRowCount() const
{
    return rows;
}
int Grid4C::getColumnCount() const
{
    return columns;
}
int Grid4C::getLevelCount() const
{
    return levels;
}

int clamp(int min, int val, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
    return val;
}