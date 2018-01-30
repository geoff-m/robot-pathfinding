//
// Created by Geoff M. on 11/17/17.
//

//#include <iostream> // for debug
#include "BMController.h"
#include <vector>
#include <string>
#include "message_filters/subscriber.h"

#define DEFAULT_COST 0 // should be 1? idk. just check dstarlite's default cost for a node.
#define NONTRAVERSABLE_COST -1
#define MINIMUM_SAFE_DISTANCE 2

void BMController::registerCallbacks(const char* baseName, int robotCount)
{
    for (int id = 0; id < robotCount; ++id)
    {
        std::string rosName(baseName);
        std::string trueName(baseName);
        rosName.append('_' + std::to_string(id));
        trueName.append('#' + std::to_string(id));

        if (strcmp(trueName.c_str(), driver->getName()) == 0)
        {
            altPublisher = nh->advertise<geometry_msgs::Polygon>(rosName + "/out/alternatives", 1, false);

            // Go ahead and subscribe to own-robot's position as well.
            ////std::printf("Skipping registering callback for \"%s\" because it has the same name as my driver.\n", trueName.c_str());
            ////continue;
        }

        std::string locationTopic = rosName + "/out/location";
        std::printf("Controller is registering callback for \"%s\"...\n", locationTopic.c_str());
        /*locationSubscribers.push_back( nh->subscribe(locationTopic,
                                                     1,
                                                     &BMController::locationCallback, this));
                                                     */
        locationSubscribers.emplace_back(new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, locationTopic, 1));
        locationSubscribers.back()->registerCallback(&BMController::locationCallback, this);

        std::string alternativeTopic = rosName + "/out/alternatives";
        std::printf("Controller is registering callback for \"%s\"...\n", alternativeTopic.c_str());
        altSubscribers.emplace_back(new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, alternativeTopic, 1));
        altSubscribers.back()->registerCallback(&BMController::alternativeCallback, this);
        /*altSubscribers.push_back(nh-> subscribe(alternativeTopic,
                                                1,
                                                &BMController::alternativeCallback, this));
                                                */


    }
}

void BMController::locationCallback(const geometry_msgs::Polygon& msg)
{
    //std::cout << "HIT CALLBACK" << std::endl;
    int id = (int)(msg.points[0].x);
    auto loc = msg.points[1];
    PointD3D worldLoc(loc.x, loc.y, loc.z);
    Point3D gridLoc = grid.get()->getGridPoint(worldLoc);
    std::cout << id << "'s location: [" << loc.x << ", " << loc.y << ", " << loc.z << "] = [" <<
              gridLoc.getX() << ", " << gridLoc.getY() << ", " << gridLoc.getZ() << "]" << std::endl;


    // Update our knowledge of that robot's location.
    std::lock_guard<std::mutex> lock(robotLocations_mutex); // change this to using a timed mutex?
    auto it = robotLocations.find(id);
    if (it == robotLocations.end())
    {
        robotLocations.insert(std::make_pair(id, gridLoc));
    } else {
        if (!(it->second == gridLoc)) // Check if this constitutes a change in grid position.
        {
            it->second = gridLoc;
            robotLocationChanged(it->first); // check if coordination is necessary, and do so if it is.
            // We call robotLocationChanged only when at least one grid position has changed for any known robot (including self).
            // This dramatically cuts down the calls to this function which tests for minimum safe distance.
        }
    }
}

BMController::~BMController()
{
    std::cout << "Destroying controller.\n";
    for (auto& sub : locationSubscribers) {
        //sub.shutdown();
        sub->unsubscribe();
    }
    for (auto& sub : altSubscribers) {
        //sub.shutdown();
        sub->unsubscribe();
    }
    robotLocations.clear();
}

static list<Point3D> fromState(list<state> s);

void BMController::markColumnNontraversable(int rowStart, int rowEnd, int col)
{
    for (int row = rowStart; row < rowEnd; ++row)
    {
        dstar.updateCell(row, col, NONTRAVERSABLE_COST);
    }
}

void BMController::markRowNontraversable(int row, int colStart, int colEnd)
{
    for (int col = colStart; col < colEnd; ++col)
    {
        dstar.updateCell(row, col, NONTRAVERSABLE_COST);
    }
}

void BMController::navigateTo(int row, int col)
{
    // Plan the path.
    Point3D myGridLoc = grid->getGridPoint(*(driver->myLoc.get()));
    dstar.init(myGridLoc.getX(), myGridLoc.getY(), row, col);
    dstar.replan();
    list<state> path = dstar.getPath();
    list<Point3D> waypoints = fromState(path);
    auto worldPath = grid->getWorldPoints(waypoints);

    // Tell the driver to follow the path.
    driver->followPath(worldPath);
}

static list<Point3D> fromState(list<state> s)
{
    list<Point3D> ret;
    for (auto iter = s.begin();
         iter != s.end();
         ++iter)
    {
        state current = *iter;
        Point3D p(current.x, current.y, 0);
        ret.push_back(p);
    }
    return ret;
}

// todo: make each controller subscribe to every other robot's out/location topic
// this will necessitate the messages from VREP being changed to include an ID from each robot
// (since they will all proc the same callback).

// todo: when a robot is within minimum safe distance, enter coordinating state.


void BMController::robotLocationChanged(int id)
{
    if (currentState != FOLLOWING_PATH)
        return;
    // Get my grid point
    // Point3D myGridLoc = grid->getGridPoint(*driver->myLoc);
    int myID = driver->getID();
    Point3D myGridLoc = robotLocations[myID];
    if (id == myID)
    {
        // We have moved, so we must recompute distances to every other robot.
        for (const auto kvp : robotLocations)
        {
            // Check distance
            // todo: SKIP COMPUTING DISTANCE TO MYSELF
            Point3D gridDifference = myGridLoc - kvp.second;
            double proximity = gridDifference.manhattanNorm();
            if (proximity < MINIMUM_SAFE_DISTANCE)
            {
                // React to nearby robot.
                setState(COORDINATING);
                //return; // We return here immediately because it only takes 1 robot within MSD to cause us to coordinate.
                robotAlternatives[kvp.first][0] = Alternative(Point3D(), -1);
            }
        }
    } else {
        // Some non-self robot has moved. We must check its distance to us.
        Point3D gridDifference = myGridLoc - robotLocations[id];
        double proximity = gridDifference.manhattanNorm();
        if (proximity < MINIMUM_SAFE_DISTANCE)
        {
            // React to nearby robot.
            setState(COORDINATING);
        }
    }
}

void BMController::setState(State newState)
{
    switch (currentState)
    {
        case REACHED_GOAL:
            switch (newState)
            {
                case REACHED_GOAL:
                    std::cout << "No state change: Reached Goal --> Reached Goal\n";
                    break;
                case FOLLOWING_PATH:
                    std::cout << "Invalid state transition: Reached Goal --> Following Path\n";
                    break;
                case COORDINATING:
                    std::cout << "Invalid state transition: Reached Goal --> Coordinating\n";
                    // break / throw exception here?
                    break;
            }
            break;
        case FOLLOWING_PATH:
            switch (newState)
            {
                case REACHED_GOAL:
                    std::cout << "Changing state: Following Path --> Reached Goal\n";
                     // todo: stop moving.
                    // stop motors.
                    driver->stop();
                    break;
                case FOLLOWING_PATH:
                    std::cout << "No state change: Following Path --> Following Path\n";
                    break;
                case COORDINATING:
                    std::cout << "Changing state: Following Path --> Coordinating\n";

                    // Stop moving.
                    driver->stop();
                    // Find (D*) and publish my alternate paths.
                    findAlternatePaths();
                    // Wait until I've received everyone else's alternatives.

                    // todo: receive next cell info from others.
                    // todo: do bipartite matching.
                    // todo: share matching.
                    // todo: choose best matching.
                    // todo: change state to FOLLOWING_PATH and follow that matching.
                    break;
            }
            break;
        case COORDINATING:
            switch (newState)
            {
                case REACHED_GOAL:
                    std::cout << "Invalid state transition: Coordinating --> Reached Goal\n";
                    break;
                case FOLLOWING_PATH:
                    std::cout << "Changing state: Coordinating --> Following Path\n";
                    // We're following the collision-avoidance path.
                    break;
                case COORDINATING:
                    std::cout << "No state change: Coordinating --> Coordinating\n";
                    break;
            }
            break;
    }
}

// Blocks until we've received alternatives from all the robots within our D_safe.
void BMController::waitForAllAlternatives()
{
    while (true)
    {

    }
}

void BMController::findAlternatePaths()
{
    int myID = driver->getID();
    Point3D self = robotLocations[myID];
    dstar.updateStart(self.getX(), self.getY());

    // First alternate path: Assume no other robots move.
    for (const auto kvp : robotLocations)
    {
        if (kvp.first != myID)
        {
            Point3D other = kvp.second;
            dstar.updateCell(other.getX(), other.getY(), NONTRAVERSABLE_COST);
        }
    }

    bool replanRet = dstar.replan();
    std::cout << "Replan for 1st alt path returned " << replanRet << std::endl;

    // Broadcast the next point and total cost of this alternate path.
    auto nextStep = dstar.getPath().begin()++; // Get the 2nd point in this path (it begins in our current location).
    // SYNTAX MAY BE WRONG HERE ^^.
    // todo: add total cost of path + distance already travelled, to the message. see page 859 top left.

    geometry_msgs::Polygon alt1;
    geometry_msgs::Point32 a1;
    a1.x = nextStep->x;
    a1.y = nextStep->y;
    alt1.points.push_back(a1);
    geometry_msgs::Point32 id1;
    id1.x = myID;
    id1.y = 1; // Indicates that this is the first of my alternative paths.
    id1.z = totalDistanceTravelled + 1; // Total cost of this alternative path. // todo: change to + cost of entire new path starting at current location
    alt1.points.push_back(id1);
    altPublisher.publish(alt1);

    // Second alternate path: Assume all robots could move anywhere.
    for (const auto kvp : robotLocations)
    {
        if (kvp.first != myID)
        {
            Point3D other = kvp.second;
            for (const auto neighbor : grid->getNeighbors(other))
            {
                dstar.updateCell(neighbor.getX(), neighbor.getY(), NONTRAVERSABLE_COST);
            }
        }
    }

    replanRet = dstar.replan();
    std::cout << "Replan for 2nd alt path returned " << replanRet << std::endl;

    // Broadcast the next point and total cost of this alternate path.
    nextStep = dstar.getPath().begin()++; // Get the 2nd point in this path (it begins in our current location).
    // SYNTAX MAY BE WRONG HERE ^^.
    geometry_msgs::Polygon alt2;
    geometry_msgs::Point32 a2;
    a2.x = nextStep->x;
    a2.y = nextStep->y;
    alt2.points.push_back(a2);
    geometry_msgs::Point32 id2;
    id2.x = myID;
    id2.y = 2; // Indicates that this is the second of my alternative paths.
    id2.z = totalDistanceTravelled + 1; // Total cost of this alternative path. // todo: change to + cost of entire new path starting at current location
    alt2.points.push_back(id2);
    altPublisher.publish(alt2);

    // Clear all the cells we just marked as nontraversable.
    // note: be careful do not clear the walls. or maybe go ahead and clear them, then call constructor methods again.
    for (const auto kvp : robotLocations)
    {
        if (kvp.first != myID)
        {
            Point3D other = kvp.second;
            dstar.updateCell(other.getX(), other.getY(), DEFAULT_COST);
            for (const auto neighbor : grid->getNeighbors(other))
            {
                dstar.updateCell(neighbor.getX(), neighbor.getY(), DEFAULT_COST);
            }
        }
    }

    // Set walls again, in case we just cleared any.
    setupWalls();
}


void BMController::alternativeCallback(const geometry_msgs::Polygon& msg)
{
    // Extract the message fields.
    int id = (int)(msg.points[0].x);
    int altNumber = (int)(msg.points[0].y);
    int cost = (int)(msg.points[0].z);
    auto loc = msg.points[1];
    Point3D gridLoc(loc.x, loc.y, loc.z);
    std::cout << id << "'s" << altNumber << (altNumber == 1 ? "st" : "nd") << "alternative: [" <<
                                                loc.x << ", " << loc.y << ", " << loc.z << "]\n";

    Alternative alt;
    alt.NextStep = gridLoc;
    alt.TotalCost = cost;

    // Update our table of alternatives.
    std::lock_guard<std::mutex> lock(robotLocations_mutex); // change this to using a timed mutex?
    // We're using the same mutex here as for the robotLocations map.
    // I think that should be okay since we won't have changing locations when alternatives are being received.
    robotAlternatives[id][altNumber - 1] = alt;
}


    /* For evasion:
     *
     * for each known nearby robot (class-level var)
     * {
     *  // Mark the robot's location as blocked.
        dstar.updateCell(location.x, location.y, NONTRAVERSABLE_COST);

       }

       compute BM (alg. 1 pp. 859)

       broadcast the resulting matching i found (latch)

       wait for receipt of all nearby robot's matchings // sic. wait as long as necessary to receive matching results

       choose best matching based on max size (break ties with min. weight)

       broadcast that choice (just for good measure)

       move according to that choice.

     *
     */


    /*
     * Give reach robot a state:
     * -Reached goal (not moving)
     * -Following path (moving)
     * -Coordinating (not moving)
     *
     * see picture on phone for 5 steps involved in coordinating
     *
     */

void BMController::setupWalls()
{
    int minCol = 0;
    int maxCol = grid->getColumnCount() - 1;
    int minRow = 0;
    int maxRow = grid->getRowCount() - 1;
    markColumnNontraversable(minRow, maxRow, minCol);
    markColumnNontraversable(minRow, maxRow, maxCol);
    markRowNontraversable(minRow, minCol, maxCol);
    markRowNontraversable(maxRow, minCol, maxCol);
}