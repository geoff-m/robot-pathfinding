//
// Created by Geoff M. on 11/8/17.
//

#include "main.h"
#include "Grid4C.h"
#include "VrepPioneerDriver.h"
#include "BMController.h"
#include "ros/ros.h"
#include <iostream>
#include <thread>
#include "Dstar.h"

static list<Point3D> fromState(list<state> s);

Countdown* activeWorkers;

int main(int argc, char *argv[]) {
    ROS_INFO("Calling rosinit\n");
    ros::init(argc, argv, "pathdriver");

    const PointD3D gridOrigin(0.0f, 0.0f, 0.0f);

    ROS_DEBUG("Constructing grid\n");
    const float GRID_SCALE = 2.0f;
    std::shared_ptr<Grid4C> grid(new Grid4C(gridOrigin, 25, 25, 1, GRID_SCALE, GRID_SCALE, GRID_SCALE));

    ROS_DEBUG("Constructing node\n");
    ros::NodeHandle* node = new ros::NodeHandle();

    ROS_DEBUG("Constructing %d driver(s)\n", ROBOT_COUNT);

    VrepPioneerDriver* drivers[ROBOT_COUNT];
    //drivers[0] = new VrepPioneerDriver(*node, "Pioneer_p3dx#0");
    //drivers[1] = new VrepPioneerDriver(*node, "Pioneer_p3dx#1");
    //drivers[2] = new VrepPioneerDriver(*node, "Pioneer_p3dx#2");
    for (int i=0; i < ROBOT_COUNT; ++i)
    {
        drivers[i] = new VrepPioneerDriver(*node, "Pioneer_p3dx#" + to_string(i));
    }


    // Start processing ROS callbacks and allow time for sensor fields to be set before we attempt navigation, etc.
    ros::AsyncSpinner spinner(0);
    spinner.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // If you get a crash after this point but before any robot moves, it may be because:
    //      One or more robots don't exist (check names).
    //      ROS is not started.
    //      V-REP simulation is not running.

    //drivers[0]->driveTo(grid.get()->getWorldPoint(Point3D(0, 0, 0)));
    //drivers[1]->driveTo(grid.get()->getWorldPoint(Point3D(0, 1, 0)));
    //drivers[2]->driveTo(grid.get()->getWorldPoint(Point3D(0, 2, 0)));

    ROS_DEBUG("Constructing %d controllers (one for each driver)\n", ROBOT_COUNT);
    BMController* controllers[ROBOT_COUNT];
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        controllers[i] = new BMController(drivers[i], // The driver for the robot to be associated with this controller.
                                          grid.get(), // The grid that the controller will navigate in.
                                          *node, // The ROS NodeHandle.
                                          "Pioneer_p3dx", // The base name of the robot.
                                          ROBOT_COUNT); // Total number of robots in the scene.
    }
    for (int i = 0; i < ROBOT_COUNT; ++i) {
        PointD3D actualLoc = *drivers[i]->myLoc;
        Point3D nearestGridLoc = grid->getGridPoint(actualLoc);
        PointD3D worldGridLoc = grid->getWorldPoint(nearestGridLoc);
        double err = (worldGridLoc - actualLoc).euclideanNorm();
        const double WORST_CASE = GRID_SCALE / sqrt(2);
        std::printf("Robot %d is initially %.2f (%.1f%% of worst case) away from nearest grid location, (%d, %d).\n", i,
                    err,
                    100 * err / WORST_CASE,
                    nearestGridLoc.getX(), nearestGridLoc.getY());

        drivers[i]->driveTo(worldGridLoc);
    }

    //activeWorkers = new Semaphore(ROBOT_COUNT);
    activeWorkers = new Countdown(1);

    printf("Setup done.\n\n");

    // later, this will be made to run on its own thread (1 thread per robot)
    // update: or we can dispense with threading and use separate processes for multiple robots.



    controllers[0]->navigateTo(1, 6);
    //controllers[1]->navigateTo(1, 4);

    activeWorkers->wait();

    std::cout << "Exiting pathdriver application." << endl;
    spinner.stop();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // We must destroy all controllers, and then all drivers, to avoid crash on exit.
    // Removing these will give us SIGABRT from boost or glibc.
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        delete controllers[i];
    }
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        delete drivers[i];
    }
}