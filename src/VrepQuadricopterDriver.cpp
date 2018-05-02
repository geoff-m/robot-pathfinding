//
// Created by student on 4/19/18.
//

#include "VrepQuadricopterDriver.h"
#include <thread>
#include <regex>

VrepQuadricopterDriver::VrepQuadricopterDriver(ros::NodeHandle& node,
                                     const std::string robotName) : VrepQuadricopterDriver(node, robotName, "target")
{ }

VrepQuadricopterDriver::VrepQuadricopterDriver(ros::NodeHandle& node,
                                     const std::string robotName,
                                     const std::string targetName)
{
    std::cout << "Setting up driver for " << robotName << "...\n";
    nh = &node;
    /*if (nh.get() == nullptr)
        nh.reset(&node);
        */
    this->robotName = robotName.c_str();
    this->targetName = targetName.c_str();
    initTopics();
    canGo = true;
}


void VrepQuadricopterDriver::initTopics()
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
    locationSubscriber = new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, topicName, 1);
    locationSubscriber->registerCallback(&VrepQuadricopterDriver::locationCallback, this);

    topicName = topicNameBase + "/in/" + targetName;
    targetPublisher = nh->advertise<geometry_msgs::Vector3>(topicName, 1, true); // true for latching behavior.

    std::cout << "Driver's locationSubscriber's topic: " << locationSubscriber->getTopic() << std::endl;

    std::cout << "Driver's targetPublisher's topic: " << targetPublisher.getTopic() << std::endl;

}

void VrepQuadricopterDriver::enableMovement()
{
    std::lock_guard<std::mutex> lock(canGoMutex);
    canGo = true;
}

void VrepQuadricopterDriver::disableMovement()
{
    std::lock_guard<std::mutex> lock(canGoMutex);
    canGo = false;
    //stop();
}

bool VrepQuadricopterDriver::isMovementEnabled() const
{
    std::lock_guard<std::mutex> lock(canGoMutex);
    return canGo;
}


bool VrepQuadricopterDriver::driveTo(PointD3D target) const
{
    //std::cout << "location has " << locationSubscriber.getNumPublishers() << " publishers" << std::endl;
    //std::cout << "Driving to " << target << std::endl;
    std::printf("Robot %d:\tDriving to (world) (%.1f, %.1f)\n", id, target.getX(), target.getY());
    std::this_thread::sleep_for(std::chrono::seconds(2)); // to help keep UAV's speed low.

    const double MAX_ERROR = 0.25;
    PointD3D difference = target - *myLoc;
    double error = difference.euclideanNorm();
    if (error > MAX_ERROR)
    {
        geometry_msgs::Vector3 msg;
        msg.x = target.getX();
        msg.y = target.getY();
        msg.z = target.getZ();
        targetPublisher.publish(msg);
    }
    clock_t time = clock();
    while (error > MAX_ERROR)
    {
        // Check that we haven't been signalled to stop.
        bool willGo;
        canGoMutex.lock();
        willGo = canGo;
        canGoMutex.unlock();
        if (!willGo)
        {
            std::printf("Driver %d:\tStopping driving because canGo has been cleared!\n", id);
            return false;
        }

        ros::spinOnce(); // wait for position change
        difference = target - *myLoc;
        error = difference.euclideanNorm();

        if (time - clock() > 1000)
        {
            time = clock();
            std::cout << "Current error distance: " << error << std::endl;
        }
    }
    return true;
}

void VrepQuadricopterDriver::followPath(std::list<PointD3D> waypoints) const
{
    ROS_INFO("entered followPath");
    for (auto iter = waypoints.begin();
         iter != waypoints.end();
         ++iter)
    {
        driveTo(*iter);
    }
}

void VrepQuadricopterDriver::stop() const
{
    driveTo(*myLoc);
}

void VrepQuadricopterDriver::locationCallback(const geometry_msgs::Polygon& msg)
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



const int VrepQuadricopterDriver::getID() const
{
    return id;
}

bool VrepQuadricopterDriver::faceDirection(double degrees) const
{
    // Not supported for Quadricopter.
    return true;
}

void VrepQuadricopterDriver::turnLeft(float speed) const
{
    // Not supported for Quadricopter.
}

void VrepQuadricopterDriver::turnRight(float speed) const
{
    // Not supported for Quadricopter.
}

void VrepQuadricopterDriver::goForward(float speed) const
{
    // Not supported for Quadricopter.
}

const char* VrepQuadricopterDriver::getName() const
{
    return robotName;
}


PointD3D VrepQuadricopterDriver::getLocation () const
{
    return *myLoc;
}

PointD3D VrepQuadricopterDriver::getOrientation() const
{
    __glibcxx_assert(false);
    return PointD3D(-1, -1, -1);
}

VrepQuadricopterDriver::~VrepQuadricopterDriver()
{
    std::cout << "Destroying driver.\n";
    if (targetPublisher)
        targetPublisher.shutdown();
    /*if (locationSubscriber)
        locationSubscriber.shutdown();
        */
    locationSubscriber->unsubscribe();
}