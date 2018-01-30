// Navigates a single robot in a multi-robot setting, using the algorithm described in Dutta.
// Created by Geoff M. on 11/17/17.
//

#ifndef PATHDRIVER_BMPLANNER_H
#define PATHDRIVER_BMPLANNER_H

#include "main.h" // for ROBOT_COUNT.

#include "Dstar.h"
#include "Grid4C.h"
#include "RobotDriver.h"
#include <memory>
#include "VrepPioneerDriver.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Polygon.h"
#include <map>
#include <thread>
#include "message_filters/subscriber.h"
#include "Alternative.h"

// This class contains the beliefs and knowledge of a single robot.
class BMController {

private:
    int altsWaiting; // The number of robots I'm waiting to hear alternatives from.

    int totalDistanceTravelled; // Measured in cells.

    enum State {
        REACHED_GOAL,
        FOLLOWING_PATH,
        COORDINATING
    };

    State currentState;
    void setState(State newState);

    Dstar dstar;
    void markColumnNontraversable(int rowStart, int rowEnd, int col);
    void markRowNontraversable(int row, int colStart, int colEnd);

    VrepPioneerDriver* driver;
    std::shared_ptr<Grid4C> grid;

    ros::Publisher altPublisher; // Publisher of alternative path data.
    /* Format: geometry_msgs::Polygon
     * First point: x = this robot's id
     *              y = which alternative, 1 or 2
     *              z = total cost of alternative path
     * Second point: The next point proposed by the alternative path.
     */
    vector<message_filters::Subscriber<geometry_msgs::Polygon>*> locationSubscribers;
    vector<message_filters::Subscriber<geometry_msgs::Polygon>*> altSubscribers;
    //vector<ros::Subscriber> locationSubscribers;
    //vector<ros::Subscriber> altSubscribers;

    void waitForAllAlternatives();

    Alternative robotAlternatives[ROBOT_COUNT][2]; // Holds two alternatives for each robot.
    // Let's say that robotAlternatives[i][0]=robotAlternatives[i][1]=Alternative{0, -1} indicates we're waiting on robot i's alternatives.

    ros::NodeHandle* nh;
    void registerCallbacks(const char* baseName, int robotCount);
    void locationCallback(const geometry_msgs::Polygon& msg);
    void alternativeCallback(const geometry_msgs::Polygon& msg);

    std::mutex robotLocations_mutex;
    std::map<int, Point3D> robotLocations; // Associative array indicating the the robot locations we know about.

    /** Called when a we're informed a robot's location has changed.
     *
     * @param id The ID of the robot whose location has been updated in the map.
     */
    void robotLocationChanged(int id);

    void findAlternatePaths();

    void setupWalls();

public:
    BMController(VrepPioneerDriver* driver,
                 Grid4C* g,
                 ros::NodeHandle& node,
                 const char* robotBaseName, // The name of the robot without any ID appended.
                 int robotCount) // IDs range from 0 to robotCount-1.
    {
        this->driver = driver;
        //this->driver.reset(driver);
        //this->grid = grid;
        grid.reset(g);

        nh = &node;

        // Mark the borders of the grid as nontraversable for D*.
        // We assume only 2-dimensional case at this point.
        setupWalls();

        totalDistanceTravelled = 0;

        registerCallbacks(robotBaseName, robotCount);
    }

    // Plans a path and begins driving to the given location.
    void navigateTo(int row, int col);

    ~BMController();
};


#endif //PATHDRIVER_BMPLANNER_H
