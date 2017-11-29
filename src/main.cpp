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

int main(int argc, char *argv[]) {
    ROS_INFO("calling rosinit\n");
    ros::init(argc, argv, "pathdriver");

    const PointD3D gridOrigin(0.0f, 0.0f, 0.0f);

    ROS_DEBUG("constructing grid\n");
    std::shared_ptr<Grid4C> grid(new Grid4C(gridOrigin, 25, 25, 1, 1.0f, 1.0f, 0.0f));

    ROS_DEBUG("constructing node\n");
    ros::NodeHandle* node = new ros::NodeHandle();

    ROS_DEBUG("constructing driver\n");
    VrepPioneerDriver driver1(*node, "Pioneer_p3dx");
    //VrepPioneerDriver driver2(*node, "Pioneer_p3dx#0");

    // Start processing ROS callbacks and allow time for sensor fields to be set before we attempt navigation, etc.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    BMController controller1(&driver1, grid.get());

    controller1.navigateTo(2, 2);

    //driver.followPath(worldPath); // invoke this on its own thread (a new one)


    /*
    driver.stop();

    driver.driveTo(grid.getWorldPoint(Point3D(0, 0, 0)));
    driver.driveTo(grid.getWorldPoint(Point3D(4, 0, 0)));
     */

    std::cout << "Exiting pathdriver." << endl;
}
