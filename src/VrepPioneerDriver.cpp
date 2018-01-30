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
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Vector3.h"
#include "geometry.h"
#include <time.h>
#include <regex>
#include "message_filters/subscriber.h"

//ros::NodeHandle* nh;
//ros::Subscriber locationSubscriber, orientationSubscriber;


VrepPioneerDriver::VrepPioneerDriver(ros::NodeHandle& node,
                                    const std::string robotName)
{
    std::cout << "Setting up driver for " << robotName << "...\n";
    nh = &node;
    /*if (nh.get() == nullptr)
        nh.reset(&node);
        */
    this->robotName = robotName.c_str();
    this->leftMotorName = "leftMotor";
    this->rightMotorName = "rightMotor";
    initTopics();
}

VrepPioneerDriver::VrepPioneerDriver(ros::NodeHandle& node,
                                     const std::string robotName,
                                     const std::string leftMotorName,
                                     const std::string rightMotorName)
{
    std::cout << "Setting up driver for " << robotName << "...\n";
    nh = &node;
    /*if (nh.get() == nullptr)
        nh.reset(&node);
        */
    this->robotName = robotName.c_str();
    this->leftMotorName = leftMotorName.c_str();
    this->rightMotorName = rightMotorName.c_str();
    initTopics();
}

const int VrepPioneerDriver::getID() const
{
    return id;
}

void VrepPioneerDriver::initTopics()
{
    // Sanitize robot name. For example, "myRobot#25" will become "myRobot_25".
    std::string topicNameBase;
    std::string topicNameSuffix;
    std::regex reg("([a-zA-Z0-9_/]*)#([0-9]+)");
    std::cmatch match;
    if (std::regex_match(robotName, match, reg))
    {
        /*for (int i=0; i<sm.size(); ++i)
        {
            std::cout << "sm[" << i << "] = " << sm[i] << std::endl;
        }*/
        topicNameSuffix = "_";
        topicNameSuffix += match[2];
        id = std::stoi(match[2]);
        topicNameBase = match[1];
        topicNameBase += topicNameSuffix;
    } else {
        throw std::runtime_error("Robot name has invalid format.");
        topicNameBase = robotName;
        topicNameSuffix = "";
    }


    std::string topicName = topicNameBase + "/out/location";

    // Subscribe with the callback being this instance's locationCallback function.
    /*locationSubscriber = nh->subscribe(topicName,
                                       1,
                                       &VrepPioneerDriver::locationCallback, this);
    */
    locationSubscriber = new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, topicName, 1);
    locationSubscriber->registerCallback(&VrepPioneerDriver::locationCallback, this);

    topicName = topicNameBase + "/out/orientation";
    /*orientationSubscriber= nh->subscribe(topicName,
                                         1,
                                         &VrepPioneerDriver::orientationCallback, this);
    */
    orientationSubscriber = new message_filters::Subscriber<geometry_msgs::Vector3>(*nh, topicName, 1);
    orientationSubscriber->registerCallback(&VrepPioneerDriver::orientationCallback, this);
    // I think this is wrong because this.orientationCallback is only one piece of code,
    // regardless of how many instances we may have at runtime.
    // Look into ros "message filters"?

    topicName = topicNameBase + "/in/leftMotor";
    leftMotorPublisher = nh->advertise<std_msgs::Float32>(topicName, 1, true); // true for latching behavior.

    topicName = topicNameBase + "/in/rightMotor";
    rightMotorPublisher = nh->advertise<std_msgs::Float32>(topicName, 1, true); // true for latching behavior.


    std::cout << "Driver's locationSubscriber's topic: " << locationSubscriber->getTopic() << std::endl;

    std::cout << "Driver's orientationSubscriber's topic: " << orientationSubscriber->getTopic() << std::endl;

    std::cout << "Driver's leftMotorPublisher's topic: " << leftMotorPublisher.getTopic() << std::endl;

    std::cout << "Driver's rightMotorPublisher's topic: " << rightMotorPublisher.getTopic() << std::endl;

    std::cout << "\n";
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
        double direction = radiansToDegrees(atan2(difference.getY(), difference.getX()));
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
    double degHeading = radiansToDegrees(myRot->getZ());
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

        degHeading = radiansToDegrees(myRot->getZ());
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

void VrepPioneerDriver::locationCallback(const geometry_msgs::Polygon& msg)
{
    ROS_DEBUG("IN DRIVER LOCATION CALLBACK");
    std::lock_guard<std::mutex> lock(myLocation_mutex);

    //myLoc.reset(new PointD3D(msg.x, msg.y, msg.z));

    //myLoc = new PointD3D(msg.x, msg.y, msg.z);

    // points[0].x stores the robot id.
    // points[1] stores the acutal location we're interested in.
    myLoc = std::unique_ptr<PointD3D>(new PointD3D(msg.points[1].x, msg.points[1].y, msg.points[1].z));


    //ROS_INFO("Location updated: [%.2f, %.2f, %.2f]", myLoc->x, myLoc->y, myLoc->z);
}


void VrepPioneerDriver::orientationCallback(const geometry_msgs::Vector3& msg)
{
    ROS_DEBUG("IN DRIVER ORIENTATION CALLBACK");
    std::lock_guard<std::mutex> lock(myOrientation_mutex);

    myRot = std::unique_ptr<PointD3D>(new PointD3D(msg.x, msg.y, msg.z));

    //ROS_INFO("Orientation updated: [%f, %f, %f]", myRot->x, myRot->y, myRot->z);

    // (Mutex is released automatically when 'lock' leaves scope.)
}


VrepPioneerDriver::~VrepPioneerDriver()
{
    std::cout << "Destroying driver.\n";
    if (leftMotorPublisher)
        leftMotorPublisher.shutdown();
    if (rightMotorPublisher)
        rightMotorPublisher.shutdown();
    /*if (locationSubscriber)
        locationSubscriber.shutdown();
    if (orientationSubscriber)
        orientationSubscriber.shutdown();
        */
    locationSubscriber->unsubscribe();
    orientationSubscriber->unsubscribe();
}

const char* VrepPioneerDriver::getName() const
{
    return robotName;
}