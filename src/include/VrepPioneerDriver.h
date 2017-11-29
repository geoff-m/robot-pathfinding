//
// Created by Geoff M. on 11/8/17.
//
#pragma once
#ifndef PATHDRIVER_VREPPIONEERDRIVER_H
#define PATHDRIVER_VREPPIONEERDRIVER_H
#include <string>
#include "PointD3D.h"
#include <list>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <mutex>
#include <memory>
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include <condition_variable>
#include "RobotDriver.h"

class VrepPioneerDriver : public RobotDriver {
private:

    // These will be used to form ROS topic names.
    std::string robotName;
    std::string leftMotorName;
    std::string rightMotorName;

    void initTopics();

    std::mutex myLocation_mutex;
    std::mutex myOrientation_mutex;
    void locationCallback(const geometry_msgs::Point& msg);
    void orientationCallback(const geometry_msgs::Vector3& msg);

public:
    VrepPioneerDriver(ros::NodeHandle& node,
            const std::string robotName);

    VrepPioneerDriver(ros::NodeHandle& node,
                      const std::string robotName,
                      const std::string leftMotorName,
                      const std::string rightMotorName);

    void driveTo(PointD3D target) const;
    void followPath(std::list<PointD3D> waypoints) const;

    void faceDirection(double angle) const;
    void goForward(float speed) const;
    void stop() const;
    void turnLeft(float speed) const;
    void turnRight(float speed) const;

    std::unique_ptr<PointD3D> myLoc;
    std::unique_ptr<PointD3D> myRot;

    ~VrepPioneerDriver();
};


#endif //PATHDRIVER_VREPPIONEERDRIVER_H
