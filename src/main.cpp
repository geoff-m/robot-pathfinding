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
#include "VrepQuadricopterDriver.h"

static list<Point3D> fromState(list<state> s);

Countdown* activeWorkers;

int quadricopterMain();

int main(int argc, char *argv[]) {

    ROS_INFO("Calling rosinit\n");
    ros::init(argc, argv, "pathdriver");
    //consoleMutex = std::make_shared<std::mutex>();
    return quadricopterMain();

    Log logger("/home/student/pathfinding-tests/pioneer", ROBOT_COUNT);

    const PointD3D gridOrigin(0.0f, 0.0f, 0.0f);

    ROS_DEBUG("Constructing grid\n");
    // 0.8 is too small
    // 1.5 is too small??
    const float GRID_SCALE = 0.5f;
    std::shared_ptr<Grid4C> grid(new Grid4C(gridOrigin, ROW_COLUMN_COUNT, ROW_COLUMN_COUNT, 1, GRID_SCALE, GRID_SCALE, GRID_SCALE));

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

    ROS_DEBUG("Constructing %d controllers (one for each driver)\n", ROBOT_COUNT);
    BMController* controllers[ROBOT_COUNT];
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        controllers[i] = new BMController(drivers[i], // The driver for the robot to be associated with this controller.
                                          grid.get(), // The grid that the controller will navigate in.
                                          *node, // The ROS NodeHandle.
                                          "Pioneer_p3dx", // The base name of the robot.
                                          ROBOT_COUNT, // Total number of robots in the scene.
                                          &logger);
    }


    activeWorkers = new Countdown(ROBOT_COUNT);

    printf("Setup done.\n\n");

    // later, this will be made to run on its own thread (1 thread per robot)
    // update: or we can dispense with threading and use separate processes for multiple robots.


    controllers[0]->navigateTo(4, 17);
    controllers[1]->navigateTo(4, 8);


    std::cout << "Main: Waiting for robots to finish...\n";
    activeWorkers->wait();

    std::cout << "Exiting pathdriver application.\n";
    spinner.stop();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // We must destroy all controllers, and then all drivers, to avoid crash on exit.
    // Removing these will give us SIGABRT from boost or glibc.
        for (int i = 0; i < ROBOT_COUNT; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::printf("deleting controller %d\n", i);

            delete controllers[i];
        }
        for (int i = 0; i < ROBOT_COUNT; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::printf("deleting driver %d\n", i);
            delete drivers[i];
        }

}

void traceBoundary(Grid4C* grid, VrepQuadricopterDriver* driver) {
    int x = grid->getRowCount();
    int y = grid->getColumnCount();
    int z = grid->getLevelCount();

    while (true)
    {
        driver->driveTo(grid->getWorldPoint(Point3D(0, 0, 0)));
        driver->driveTo(grid->getWorldPoint(Point3D(x, 0, 0)));
        driver->driveTo(grid->getWorldPoint(Point3D(x, y, 0)));
        driver->driveTo(grid->getWorldPoint(Point3D(x, y, z)));
        driver->driveTo(grid->getWorldPoint(Point3D(0, y, z)));
        driver->driveTo(grid->getWorldPoint(Point3D(0, 0, z)));
        driver->driveTo(grid->getWorldPoint(Point3D(x, 0, z)));
        driver->driveTo(grid->getWorldPoint(Point3D(0, 0, 0)));
        driver->driveTo(grid->getWorldPoint(Point3D(0, y, 0)));
    }
}

int quadricopterMain()
{
    std::string logPath("/home/student/pathfinding-tests/uav/");
    logPath.append(to_string(ROBOT_COUNT));
    logPath.append("r");
    Log logger(logPath, ROBOT_COUNT);
    const PointD3D gridOrigin(-2.0f, -2.0f, 0.5f);

    ROS_DEBUG("Constructing grid\n");
    const float GRID_SCALE = 0.8f;
    std::shared_ptr<Grid4C> grid(new Grid4C(gridOrigin, ROW_COLUMN_COUNT, ROW_COLUMN_COUNT, 1, GRID_SCALE, GRID_SCALE, GRID_SCALE));

    ROS_DEBUG("Constructing node\n");
    ros::NodeHandle* node = new ros::NodeHandle();

    ROS_DEBUG("Constructing %d driver(s)\n", ROBOT_COUNT);

    VrepQuadricopterDriver* drivers[ROBOT_COUNT];
    for (int i=0; i < ROBOT_COUNT; ++i)
    {
        drivers[i] = new VrepQuadricopterDriver(*node, "Quadricopter#" + to_string(i));
    }

    // Start processing ROS callbacks and allow time for sensor fields to be set before we attempt navigation, etc.
    ros::AsyncSpinner spinner(0);
    spinner.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // If you get a crash after this point but before any robot moves, it may be because:
    //      One or more robots don't exist (check names).
    //      ROS is not started.
    //      V-REP simulation is not running.


    //traceBoundary(grid.get(),drivers[0]);


    for (int i = 0; i < ROBOT_COUNT; ++i) {
        PointD3D actualLoc = *drivers[i]->myLoc;
        Point3D nearestGridLoc = grid->getGridPoint(actualLoc);
        PointD3D worldGridLoc = grid->getWorldPoint(nearestGridLoc);
        double err = (worldGridLoc - actualLoc).euclideanNorm();
        const double WORST_CASE = GRID_SCALE / sqrt(3);
        std::printf("Robot %d is initially %.2f (%.1f%% of worst case) away from nearest grid location, (%d, %d, %d).\n", i,
                    err,
                    100 * err / WORST_CASE,
                    nearestGridLoc.getX(), nearestGridLoc.getY(), nearestGridLoc.getZ());

        drivers[i]->driveTo(worldGridLoc);
    }

    ROS_DEBUG("Constructing %d controllers (one for each driver)\n", ROBOT_COUNT);
    BMController* controllers[ROBOT_COUNT];
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        controllers[i] = new BMController(drivers[i], // The driver for the robot to be associated with this controller.
                                          grid.get(), // The grid that the controller will navigate in.
                                          *node, // The ROS NodeHandle.
                                          "Quadricopter", // The base name of the robot.
                                          ROBOT_COUNT, // Total number of robots in the scene.
                                          &logger);
    }

    activeWorkers = new Countdown(ROBOT_COUNT);

    printf("Setup done.\n\n");

    // later, this will be made to run on its own thread (1 thread per robot)
    // update: or we can dispense with threading and use separate processes for multiple robots.


    //consoleMutex = new std::mutex();

    controllers[0]->navigateTo(3, 5);
    controllers[1]->navigateTo(3, 0);
    //controllers[2]->navigateTo(8, 0);


    std::cout << "Main: Waiting for robots to finish...\n";
    activeWorkers->wait();

    std::cout << "Exiting pathdriver application.\n";
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

    return 0;
}