// Navigates a single robot in a multi-robot setting, using the algorithm described in Dutta.
// Created by Geoff M. on 11/17/17.
//

#ifndef PATHDRIVER_BMPLANNER_H
#define PATHDRIVER_BMPLANNER_H

#include "Dstar.h"
#include "Grid4C.h"
#include "RobotDriver.h"
#include <memory>


// This class contains the beliefs and knowledge of a single robot.
class BMController {
private:
    Dstar dstar;
    void markColumnNontraversable(int rowStart, int rowEnd, int col);
    void markRowNontraversable(int row, int colStart, int colEnd);
    RobotDriver* driver;
    std::shared_ptr<Grid4C> grid;

public:
    BMController(RobotDriver* driver, Grid4C* g)
    {
        this->driver = driver;
        //this->driver.reset(driver);
        //this->grid = grid;
        grid.reset(g);

        // Mark the borders of the grid as nontraversable for D*.
        // We assume only 2-dimensional case at this point.
        int minCol = 0;
        int maxCol = grid->getColumnCount() - 1;
        int minRow = 0;
        int maxRow = grid->getRowCount() - 1;
        markColumnNontraversable(minRow, maxRow, minCol);
        markColumnNontraversable(minRow, maxRow, maxCol);
        markRowNontraversable(minRow, minCol, maxCol);
        markRowNontraversable(maxRow, minCol, maxCol);
    }

    // Informs this BMController of another robot's presence at the specified location.
    void seeRobot(Point3D location);

    // Plans a path and begins driving to the given location.
    void navigateTo(int row, int col);
};


#endif //PATHDRIVER_BMPLANNER_H
