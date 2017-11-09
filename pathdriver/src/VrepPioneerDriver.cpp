//
// Created by Geoff M. on 11/8/17.
//

#include "include/VrepPioneerDriver.h"
#include <cmath>

#define MAX_ERROR 0.1

VrepPioneerDriver::VrepPioneerDriver(const std::string robotName)
{
    this->robotName = robotName;
    this->leftMotorName = "leftMotor";
    this->rightMotorName = "rightMotor";
}

VrepPioneerDriver::VrepPioneerDriver(const std::string robotName,
                                     const std::string leftMotorName,
                                     const std::string rightMotorName)
{
    this->robotName = robotName;
    this->leftMotorName = leftMotorName;
    this->rightMotorName = rightMotorName;
}

void VrepPioneerDriver::driveTo(PointF3D target)
{
    PointF3D myLocation = getLocation();
    PointF3D difference = target - myLocation;
    double error = difference.euclideanNorm();
    while (error > MAX_ERROR)
    {
        float direction = atan2f(difference.y, difference.x);
        goForward(1.0f);

        myLocation = getLocation();
        difference = target - myLocation;
        error = difference.euclideanNorm();
    }
    stop();
}

void VrepPioneerDriver::followPath(std::list<PointF3D> waypoints)
{
     for (std::list<PointF3D>::iterator iter = waypoints.begin();
             iter != waypoints.end();
             ++iter)
     {
         driveTo(*iter);
     }
}

// Uses ROS to turn until the robot is facing the specified direction (radians).
void VrepPioneerDriver::faceDirection(float angle)
{
    // todo
}

// Uses ROS to go forward at the specified speed.
void VrepPioneerDriver::goForward(float speed)
{
    // todo
}

// Uses ROS to stop the robot's motors.
void VrepPioneerDriver::stop()
{
   // todo
}

// Gets the robot's location via ROS.
PointF3D VrepPioneerDriver::getLocation()
{
    // todo
}