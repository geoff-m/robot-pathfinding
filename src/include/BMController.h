// Navigates a single robot in a multi-robot setting, using the algorithm described in Dutta.
// Created by Geoff M. on 11/17/17.
//

#ifndef PATHDRIVER_BMPLANNER_H
#define PATHDRIVER_BMPLANNER_H

#include "Dstar.h"
#include "Grid4C.h"
#include "RobotDriver.h"
#include <memory>
#include "VrepPioneerDriver.h"
#include "geometry_msgs/Polygon.h"


// This class contains the beliefs and knowledge of a single robot.
class BMController {
private:
    Dstar dstar;
    void markColumnNontraversable(int rowStart, int rowEnd, int col);
    void markRowNontraversable(int row, int colStart, int colEnd);
    VrepPioneerDriver* driver;
    std::shared_ptr<Grid4C> grid;

    ros::NodeHandle* nh;
    void registerCallbacks(const char* baseName, int robotCount);
    void locationCallback(const geometry_msgs::Polygon& msg);

public:
    BMController(VrepPioneerDriver* driver,
                 Grid4C* g,
                 ros::NodeHandle& node,
                 const char* robotBaseName, // The name of the robot without any ID appended.
                 int robotCount) // IDs range from 0 to robotCount-1.
    {
        this->driver = driver;
        //this->driver.reset(driver);
        //this->grid = grid;
        grid.reset(g);

        nh = &node;

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

        registerCallbacks(robotBaseName, robotCount);
    }

    // Informs this BMController of another robot's presence at the specified location.
    void seeRobot(Point3D location);

    // Plans a path and begins driving to the given location.
    void navigateTo(int row, int col);

    ~BMController();
};


#endif //PATHDRIVER_BMPLANNER_H
