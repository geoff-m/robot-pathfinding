//
// Created by Geoff M. on 11/17/17.
//

#include <iostream> // for debug
#include "BMController.h"

#define NONTRAVERSABLE_COST -1
#define MINIMUM_SAFE_DISTANCE 2

static list<Point3D> fromState(list<state> s);

void BMController::markColumnNontraversable(int rowStart, int rowEnd, int col)
{
    for (int row = rowStart; row < rowEnd; ++row)
    {
        dstar.updateCell(row, col, NONTRAVERSABLE_COST);
    }
}

void BMController::markRowNontraversable(int row, int colStart, int colEnd)
{
    for (int col = colStart; col < colEnd; ++col)
    {
        dstar.updateCell(row, col, NONTRAVERSABLE_COST);
    }
}

void BMController::navigateTo(int row, int col)
{
    bool fail = false;
    if (!grid)
    {
        std::cout << "grid is null";
        fail = true;
    }

    if (!driver)
    {
        std::cout << "driver is null";
        fail = true;
    }

    if (!(driver->myLoc))
    {
        std::cout << "driver's myLoc is null";
        fail = true;
    }

    if (fail)
        return;
    // segfault here. why?

    Point3D myGridLoc = grid->getGridPoint(*driver->myLoc);
    dstar.init(myGridLoc.x, myGridLoc.y, row, col);
    dstar.replan();
    list<state> path = dstar.getPath();
    list<Point3D> waypoints = fromState(path);
    auto worldPath = grid->getWorldPoints(waypoints);

    driver->followPath(worldPath);
}

static list<Point3D> fromState(list<state> s)
{
    list<Point3D> ret;
    for (auto iter = s.begin();
         iter != s.end();
         ++iter)
    {
        state current = *iter;
        Point3D p(current.x, current.y, 0);
        ret.push_back(p);
    }
    return ret;
}

void BMController::seeRobot(Point3D location)
{
    // Get my (approximate) grid point
    Point3D myGridLoc = grid->getGridPoint(*driver->myLoc);
    // Check distance
    Point3D gridDifference = myGridLoc - location;
    double proximity = gridDifference.manhattanNorm();
    if (proximity < MINIMUM_SAFE_DISTANCE)
    {
        // React to nearby robot.

        // add this robot to class-level list of known nearby robots.

        //// Stop this robot.
        //driver->stop();





    }
}

void evade()
{
    /*
     * for each known nearby robot (class-level var)
     * {
     *  // Mark the robot's location as blocked.
        dstar.updateCell(location.x, location.y, NONTRAVERSABLE_COST);

       }

       compute BM (alg. 1 pp. 859)

       broadcast the resulting matching i found (latch)

       wait for reciept of all nearby robot's matchings // sic. wait as long as necessary to receive matching reuslts

       choose best matching based on max size (break ties with min. weight)

       broadcast that choice (just for good measure)

       move according to that choice.

     *
     */


    /*
     * Give reach robot a state:
     * -Reached goal (not moving)
     * -Following path (moving)
     * -Coordinating (not moving)
     *
     * see picture on phone for 5 steps involved in coordinating
     *
     */
}

