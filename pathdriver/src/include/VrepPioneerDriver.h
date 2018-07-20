//
// Created by student on 11/8/17.
//

#ifndef PATHDRIVER_VREPPIONEERDRIVER_H
#define PATHDRIVER_VREPPIONEERDRIVER_H
#include <string>
#include "PointF3D.h"
#include <list>

class VrepPioneerDriver {
private:

    // These will be used to form ROS topic names.
    std::string robotName;
    std::string leftMotorName;
    std::string rightMotorName;

    PointF3D getLocation();

public:
    VrepPioneerDriver(const std::string robotName);

    VrepPioneerDriver(const std::string robotName,
                      const std::string leftMotorName,
                      const std::string rightMotorName);

    void driveTo(PointF3D target);

    void followPath(std::list<PointF3D> waypoints);

    void faceDirection(float angle);
    void goForward(float speed);
    void stop();

};


#endif //PATHDRIVER_VREPPIONEERDRIVER_H
