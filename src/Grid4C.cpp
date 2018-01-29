//
// Created by student on 11/8/17.
//

#include "Grid4C.h"
#include <stdexcept>
#include <list>

#define EMPTY_CELL -1
int clamp(int min, int val, int max);

// Represents a 6-connected 3-dimensional grid.
Grid4C::Grid4C(const PointD3D worldOrigin,
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
    int row = (int) (0.5 + (worldPoint.x - originX) / rowSpacing);
    int col = (int) (0.5 + (worldPoint.y - originY) / columnSpacing);
    int lvl = (int) (0.5 + (worldPoint.y - originZ) / levelSpacing);

    row = clamp(0, row, rows - 1);
    col = clamp(0, col, columns - 1);
    lvl = clamp(0, lvl, levels - 1);

    return Point3D(row, col, lvl);
}

PointD3D Grid4C::getWorldPoint(const Point3D gridPoint) const
{
    if (!contains(gridPoint))
    {
        throw std::invalid_argument("That point is not in the grid!");
    }
    return PointD3D(originX + gridPoint.x * rowSpacing,
                    originY + gridPoint.y * columnSpacing,
                    originZ + gridPoint.z * levelSpacing);
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
    bool top = gridPoint.z == levels;
    bool bottom = gridPoint.z == 0;
    bool front = gridPoint.x == 0;
    bool back = gridPoint.x == rows;
    bool left = gridPoint.y == 0;
    bool right = gridPoint.y == columns;
    std::list<Point3D> ret;
    if (!top)
    {
        ret.emplace_back(gridPoint.x, gridPoint.y, gridPoint.z + 1);
    }
    if (!bottom)
    {
        ret.emplace_back(gridPoint.x, gridPoint.y, gridPoint.z - 1);
    }
    if (!front)
    {
        ret.emplace_back(gridPoint.x - 1, gridPoint.y, gridPoint.z);
    }
    if (!back)
    {
        ret.emplace_back(gridPoint.x + 1, gridPoint.y, gridPoint.z);
    }
    if (!left)
    {
        ret.emplace_back(gridPoint.x, gridPoint.y - 1, gridPoint.z);
    }
    if (!right)
    {
        ret.emplace_back(gridPoint.x, gridPoint.y + 1, gridPoint.z);
    }
    return ret;
}

bool Grid4C::contains(Point3D point) const
{
    return point.x <= rows && point.x >= 0 &&
            point.y <= columns && point.y >= 0 &&
            point.z <= levels && point.z >= 0;
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