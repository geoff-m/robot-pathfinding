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
#include <vector>
#include "Matching.h"

// This class contains the beliefs and knowledge of a single robot.
class BMController {

private:
    int findBestMatchingID();
    void computeMatching();
    Matching* myMatching;

    int altsWaiting; // The number of robots I'm waiting to hear alternatives from.

    vector<int> coordinatingWith; // The IDs of the robots I'm currently involved in coordination with.

    int totalDistanceTravelled; // Measured in cells. // todo: implement me

    enum State {
        NOT_STARTED,
        REACHED_GOAL,
        FOLLOWING_PATH,
        COORDINATING
    };

    State currentState;
    void setState(State newState);

    Dstar dstar;
    void markColumnNontraversable(int rowStart, int rowEnd, int col);
    void markRowNontraversable(int row, int colStart, int colEnd);
    void setupWalls();

    VrepPioneerDriver* driver;
    std::shared_ptr<Grid4C> grid;

    ros::Publisher participatingPublisher; // Publishes a bool indicating whether or not I should be coordinated with.
    // Format: std_msgs::Bool

    ros::Publisher matchingResultPublisher; // Publishes info about the matching I found.
    // Format: geometry_msgs::Point32

    ros::Publisher fullMatchingPublisher; // Publishes my full matching, if it's the one we've decided to use.
	/* Format: geometry_msgs::Polygon
	*  Each point: 	x = where robot N should go next (x coordinate)
	* 		y = where robot N should go next (y coordinate)
	*		z = N
	*/

    void enableCoordination();
    void disableCoordination();
    bool checkIsCoordinating(int robotId); // helper method to check whether another robot should be coordinated with.

    ros::Publisher alt1Publisher; // Publisher of alternative path data.
    /* Format: geometry_msgs::Polygon
     * First point: x = this robot's id
     *              y = which alternative, 1 or 2
     *              z = total cost of alternative path
     * Second point: The next point proposed by the alternative path.
     */
    ros::Publisher alt2Publisher;
    vector<message_filters::Subscriber<geometry_msgs::Polygon>*> locationSubscribers;
    vector<message_filters::Subscriber<geometry_msgs::Polygon>*> altSubscribers;
    vector<message_filters::Subscriber<geometry_msgs::Point32>*> matchingSubscribers;
    //vector<ros::Subscriber> locationSubscribers;
    //vector<ros::Subscriber> altSubscribers;

    void waitForAllAlternatives();

    void waitForAllMatchings();

    list<state> myAlternative1, myAlternative2; // My alternative paths (entire paths).
    Alternative robotAlternatives[ROBOT_COUNT][2]; // Holds two alternatives for each robot.

    Matching robotMatchings[ROBOT_COUNT]; // Holds the matching results (just ID, cardinality, weight) from each robot.

    ros::NodeHandle* nh;
    void registerCallbacks(const char* baseName, int robotCount);
    void locationCallback(const geometry_msgs::Polygon& msg);
    void receiveAlternative(const geometry_msgs::Polygon &msg);
    void matchingCallback(const geometry_msgs::Point32& msg);

    std::mutex robotLocations_mutex;
    Point3D robotLocations[ROBOT_COUNT]; // Locations of robots we know about, indexed by ID.

    /** Called when a we're informed a robot's location has changed.
     *
     * @param id The ID of the robot whose location has been updated in the map.
     */
    void robotLocationChanged(int id);

    void findAlternatePaths();

    void transmitMatchingResult();

    void transmitFullMatching();

    Point3D awaitFullMatching(int sourceId);

    list<Point3D>* getPathFromFirstPoint(Point3D pt);

    std::string baseName;
    // This will be used to get the name of other robots we're coordinating with.
    // That is, it must be the same for all such robots.

    std::thread* driveThread;

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

        baseName = robotBaseName;

        currentState = NOT_STARTED;

        myMatching = new Matching(driver->getID());
    }

    // Drives along a list of grid points until interrupted.
    void driveAlong(list<Point3D> waypoints);

    // Plans a path and begins driving to the given location.
    void navigateTo(int row, int col);

    ~BMController();
};


#endif //PATHDRIVER_BMPLANNER_H
