//
// Created by Geoff on 4/16/18.
//

#ifndef PATHDRIVER_VREPQUADRICOPTERDRIVER_H
#define PATHDRIVER_VREPQUADRICOPTERDRIVER_H


#include <mutex>
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Vector3.h"
#include <message_filters/subscriber.h>
#include "RobotDriver.h"

class VrepQuadricopterDriver : public RobotDriver {
private:

    // These will be used to form ROS topic names.
    const char* robotName;
    const char* targetName;
    int id;

    void initTopics();

    message_filters::Subscriber<geometry_msgs::Polygon>* locationSubscriber;

    std::mutex myLocation_mutex;
    //void locationCallback(const geometry_msgs::Point& msg);
    void locationCallback(const geometry_msgs::Polygon& msg);

    ros::Publisher targetPublisher;
    //boost::shared_ptr<ros::NodeHandle> nh;
    ros::NodeHandle* nh;

    mutable std::mutex canGoMutex; // a mutex to guard canGo
    bool canGo; // Indicates whether or not driving should be allowed.

public:
    VrepQuadricopterDriver(ros::NodeHandle& node,
                      const std::string robotName);

    VrepQuadricopterDriver(ros::NodeHandle& node,
                      const std::string robotName,
                      const std::string targetName);

    bool driveTo(PointD3D target) const; // Returns true on success, false if interrupted.
    void followPath(std::list<PointD3D> waypoints) const;

    void stop() const;
    bool faceDirection(double angle) const; // not supported.
    void goForward(float speed) const; // not supported.
    void turnLeft(float speed) const; // not supported.
    void turnRight(float speed) const; // not supported.

    std::unique_ptr<PointD3D> myLoc;

    PointD3D getLocation() const;
    PointD3D getOrientation() const;


    const char* getName() const;
    const int getID() const; // Gets this robot's ID (name suffix).

    void enableMovement();
    void disableMovement();
    bool isMovementEnabled() const;

    ~VrepQuadricopterDriver();
};


#endif //PATHDRIVER_VREPQUADRICOPTERDRIVER_H
