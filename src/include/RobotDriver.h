//
// Created by Geoff M. on 11/17/17.
//

#ifndef PATHDRIVER_ROBOTDRIVER_H
#define PATHDRIVER_ROBOTDRIVER_H

#include "PointD3D.h"
#include <list>
#include <memory>

// An abstract class that represents a drivable robot.
class RobotDriver {
public:
    virtual bool driveTo(PointD3D target) const = 0;

    virtual void followPath(std::list<PointD3D> waypoints) const = 0;

    virtual void faceDirection(double angle) const = 0;
    virtual void goForward(float speed) const = 0;
    virtual void stop() const = 0;
    virtual void turnLeft(float speed) const = 0;
    virtual void turnRight(float speed) const = 0;

    std::unique_ptr<PointD3D> myLoc;
    std::unique_ptr<PointD3D> myRot;
};


#endif //PATHDRIVER_ROBOTDRIVER_H
