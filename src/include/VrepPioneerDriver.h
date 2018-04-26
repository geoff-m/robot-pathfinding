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
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Vector3.h"
#include <condition_variable>
#include "RobotDriver.h"
#include "message_filters/subscriber.h"

class VrepPioneerDriver : public RobotDriver {
private:

    // These will be used to form ROS topic names.
    const char* robotName;
    const char* leftMotorName;
    const char* rightMotorName;
    int id;

    void initTopics();

    message_filters::Subscriber<geometry_msgs::Polygon>* locationSubscriber;
    message_filters::Subscriber<geometry_msgs::Vector3>* orientationSubscriber;

    std::mutex myLocation_mutex;
    std::mutex myOrientation_mutex;
    //void locationCallback(const geometry_msgs::Point& msg);
    void locationCallback(const geometry_msgs::Polygon& msg);
    void orientationCallback(const geometry_msgs::Vector3& msg);

    ros::Publisher leftMotorPublisher, rightMotorPublisher;
    //boost::shared_ptr<ros::NodeHandle> nh;
    ros::NodeHandle* nh;

    mutable std::mutex canGoMutex; // a mutex to guard canGo
    bool canGo; // Indicates whether or not driving should be allowed.

public:
    VrepPioneerDriver(ros::NodeHandle& node,
            const std::string robotName);

    VrepPioneerDriver(ros::NodeHandle& node,
                      const std::string robotName,
                      const std::string leftMotorName,
                      const std::string rightMotorName);

    bool driveTo(PointD3D target) const; // Returns true on success, false if interrupted.
    void followPath(std::list<PointD3D> waypoints) const;

    bool faceDirection(double angle) const;
    void goForward(float speed) const;
    void stop() const;
    void turnLeft(float speed) const;
    void turnRight(float speed) const;

    PointD3D getLocation() const;
    PointD3D getOrientation() const;

    std::unique_ptr<PointD3D> myLoc;
    std::unique_ptr<PointD3D> myRot;

    const char* getName() const;
    const int getID() const; // Gets this robot's ID (name suffix).

    void enableMovement();
    void disableMovement();
    bool isMovementEnabled() const;

    ~VrepPioneerDriver();
};


#endif //PATHDRIVER_VREPPIONEERDRIVER_H
