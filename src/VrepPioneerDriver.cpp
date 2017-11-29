//
// Created by Geoff M. on 11/8/17.
//

#include "include/VrepPioneerDriver.h"
#include <cmath>
#include <mutex>
#include <thread>
#include "std_msgs/Float32.h"
#include "tf2_msgs/TFMessage.h"
#include <iostream>
#include <string>
#include <memory>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry.h"
#include <time.h>
#include <regex>

boost::shared_ptr<ros::NodeHandle> nh;
//ros::NodeHandle* nh;
ros::Subscriber locationSubscriber, orientationSubscriber;
ros::Publisher leftMotorPublisher, rightMotorPublisher;

//message_filters::Subscriber<geometry_msgs::TransformStamped> locationSubscriber;
// FOUND: THIS LINE CAUSES "you must call rosinit before creating NodeHandle" ERROR

VrepPioneerDriver::VrepPioneerDriver(ros::NodeHandle& node,
                                    const std::string robotName)
{
    //nh = &node;
    if (nh.get() == nullptr)
        nh.reset(&node);
    this->robotName = robotName;
    this->leftMotorName = "leftMotor";
    this->rightMotorName = "rightMotor";
    initTopics();
}

VrepPioneerDriver::VrepPioneerDriver(ros::NodeHandle& node,
                                     const std::string robotName,
                                     const std::string leftMotorName,
                                     const std::string rightMotorName)
{
    //nh = &node;
    if (nh.get() == nullptr)
        nh.reset(&node);
    this->robotName = robotName;
    this->leftMotorName = leftMotorName;
    this->rightMotorName = rightMotorName;
    initTopics();
}

void VrepPioneerDriver::initTopics()
{
    // Sanitize robot name. For example, "myRobot#25" will become "myRobot_25".
    std::string topicNameBase;
    std::string topicNameSuffix;
    std::regex reg("([a-zA-Z0-9_/]*)#([0-9]+)");
    std::smatch sm;
    if (std::regex_match(this->robotName, sm, reg))
    {
        /*for (int i=0; i<sm.size(); ++i)
        {
            std::cout << "sm[" << i << "] = " << sm[i] << std::endl;
        }*/
        topicNameSuffix = "_";
        topicNameSuffix += sm[2];
        topicNameBase = sm[1];
        topicNameBase += topicNameSuffix;
    } else {
        topicNameBase =  robotName;
        topicNameSuffix = "";
    }


    std::string topicName = topicNameBase + "/out/location";
    //std::string topicName = "tf";  // for testing.

    // I want to call subscribe with the callback being this instance's locationCallback function.
    locationSubscriber = nh->subscribe(topicName,
                                       1,
                                       &VrepPioneerDriver::locationCallback, this);

    topicName = topicNameBase + "/out/orientation";
    orientationSubscriber= nh->subscribe(topicName,
                                         1,
                                         &VrepPioneerDriver::orientationCallback, this);

    topicName = topicNameBase + "/in/leftMotor";
    leftMotorPublisher = nh->advertise<std_msgs::Float32>(topicName, 1, true); // true for latching behavior.

    topicName = topicNameBase + "/in/rightMotor";
    rightMotorPublisher = nh->advertise<std_msgs::Float32>(topicName, 1, true); // true for latching behavior.


    std::cout << "locationSubscriber's topic: " << locationSubscriber.getTopic() << std::endl;

    std::cout << "orientationSubscriber's topic: " << orientationSubscriber.getTopic() << std::endl;

    std::cout << "leftMotorPublisher's topic: " << leftMotorPublisher.getTopic() << std::endl;

    std::cout << "rightMotorPublisher's topic: " << rightMotorPublisher.getTopic() << std::endl;
}

void VrepPioneerDriver::driveTo(PointD3D target) const
{
    //std::cout << "location has " << locationSubscriber.getNumPublishers() << " publishers" << std::endl;
    std::cout << "Driving to " << target << std::endl;
    const double MAX_ERROR = 0.15;
    PointD3D difference = target - *myLoc;
    double error = difference.euclideanNorm();
    clock_t time = clock();
    while (error > MAX_ERROR)
    {
        double direction = radiansToDegrees(atan2(difference.y, difference.x));
        faceDirection(direction);
        goForward(1.0f);

        ros::spinOnce(); // wait for position change
        difference = target - *myLoc;
        error = difference.euclideanNorm();

        if (time - clock() > 1000)
        {
            time = clock();
            std::cout << "Current error distance: " << error << std::endl;
        }
    }
    stop();
}

void VrepPioneerDriver::followPath(std::list<PointD3D> waypoints) const
{
    ROS_INFO("entered followPath");
     for (auto iter = waypoints.begin();
             iter != waypoints.end();
             ++iter)
     {
         driveTo(*iter);
     }
}

// Uses ROS to turn until the robot is facing the specified direction (degrees).
void VrepPioneerDriver::faceDirection(double degrees) const
{
    //std::cout << "facing " << degrees << std::endl;
    const double DEGREE_EPSILON = 3;
    double error;
    int ret = 0;
    double degHeading = radiansToDegrees(myRot->z);
    error = calcDegreeDifference(degHeading, degrees);

    double min = 999999;
    double absError = fabs(error);
    while (absError > DEGREE_EPSILON)
    {
        if (absError < min)
        {
            min = absError;
            //std::cout << "New minimum abs error: " << error << std::endl;
        }
        if (error < 0)
            turnLeft(2.0f);
        else
            turnRight(2.0f);

        // Tight loop: check heading again as soon as possible.

        degHeading = radiansToDegrees(myRot->z);
        error = calcDegreeDifference(degHeading, degrees);
        absError = fabs(error);
    }
    stop();

}

void VrepPioneerDriver::turnLeft(float speed) const
{
    std_msgs::Float32 zeroSpeed;
    zeroSpeed.data = 0;
    std_msgs::Float32 m;
    m.data = speed; // speed

    leftMotorPublisher.publish(zeroSpeed);
    rightMotorPublisher.publish(m);
}

void VrepPioneerDriver::turnRight(float speed) const
{
    std_msgs::Float32 zeroSpeed;
    zeroSpeed.data = 0;
    std_msgs::Float32 m;
    m.data = speed; // speed

    leftMotorPublisher.publish(m);
    rightMotorPublisher.publish(zeroSpeed);
}

// Uses ROS to go forward at the specified speed.
void VrepPioneerDriver::goForward(float speed) const
{
    //std::cout << "goForward speed=" << speed << std::endl;
    std_msgs::Float32 m;
    m.data = speed; // speed

    leftMotorPublisher.publish(m);
    rightMotorPublisher.publish(m);
}

// Uses ROS to stop the robot's motors.
void VrepPioneerDriver::stop() const
{
   goForward(0.0f);
}

void VrepPioneerDriver::locationCallback(const geometry_msgs::Point& msg)
{
    std::lock_guard<std::mutex> lock(myLocation_mutex);

    //myLoc.reset(new PointD3D(msg.x, msg.y, msg.z));

    //myLoc = new PointD3D(msg.x, msg.y, msg.z);

    myLoc = std::unique_ptr<PointD3D>(new PointD3D(msg.x, msg.y, msg.z));


    ROS_INFO("Location updated: [%.2f, %.2f, %.2f]", myLoc->x, myLoc->y, myLoc->z);

    // (Mutex is released automatically when 'lock' leaves scope.)
}

void VrepPioneerDriver::orientationCallback(const geometry_msgs::Vector3& msg)
{
    std::lock_guard<std::mutex> lock(myOrientation_mutex);
    myRot = std::unique_ptr<PointD3D>(new PointD3D(msg.x, msg.y, msg.z));

    //ROS_INFO("Orientation updated: [%f, %f, %f]", myRot->x, myRot->y, myRot->z);

    // (Mutex is released automatically when 'lock' leaves scope.)
}


VrepPioneerDriver::~VrepPioneerDriver()
{
    if (leftMotorPublisher)
        leftMotorPublisher.shutdown();
    if (rightMotorPublisher)
        rightMotorPublisher.shutdown();
    if (locationSubscriber)
        locationSubscriber.shutdown();
    if (orientationSubscriber)
        orientationSubscriber.shutdown();
}
