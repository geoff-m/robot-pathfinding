//
// Created by Geoff M. on 11/17/17.
//

//#include <iostream> // for debug
#include "BMController.h"
#include <vector>
#include <string>
#include "message_filters/subscriber.h"
#include "std_msgs/Bool.h"
#include "Header.h" // from bipartite-matching-dutta
#include <chrono>
#include <algorithm>
#include <cstdarg>

extern Countdown* activeWorkers;

#define DEFAULT_COST 1 // d*lite's default cost for a node.
#define NONTRAVERSABLE_COST -1

// static helper methods.
static int getPathCost(list<state>& path);
static void printPath(list<state> path);
static std::string vectorToString(vector<int> v);
static list<Point3D>* fromState(list<state> s);
static Point3D getSecond(list<state> path);

void BMController::registerCallbacks(const char* baseName, int robotCount)
{
    //std::printf("\nRobot %d: Entered registerCallbacks\n", ownId);

    for (int id = 0; id < robotCount; ++id)
    {
        std::string rosName(baseName);
        std::string trueName(baseName);
        rosName.append('_' + std::to_string(id));
        trueName.append('#' + std::to_string(id));

        if (id == myID)
        {
            // This is my robot. Set up publishers.

            //std::cout << "Setting up publisher for " << rosName + "/out/alternative1\n";
            alt1Publisher = nh->advertise<geometry_msgs::Polygon>(rosName + "/out/alternative1", 1, true); // true for latching
            //std::cout << "Setting up publisher for " << rosName + "/out/alternative2\n";
            alt2Publisher = nh->advertise<geometry_msgs::Polygon>(rosName + "/out/alternative2", 1, true); // true for latching

            //std::cout << "Setting up publisher for " << rosName + "/out/participating\n";
            participatingPublisher = nh->advertise<std_msgs::Bool>(rosName + "/out/participating", 1, true); // true for latching.
            // Mark myself initially as unable to coordinate.
            disableCoordination();

            //std::cout << "Setting up publisher for " << rosName + "/out/matchingResult\n";
            matchingResultPublisher = nh->advertise<geometry_msgs::Point32>(rosName + "/out/matchingResult", 1, true);

            //std::cout << "Setting up publisher for " << rosName + "/out/fullMatching\n";
            fullMatchingPublisher = nh->advertise<geometry_msgs::Polygon>(rosName + "/out/fullMatching", 1, true);

        } else {
            // This is a non-self robot.

            // Set up alternative1 subscriber.
            /* !!! Now we use WaitForMessage instead of creating these subscribers...
             * std::string alternative1Topic = rosName + "/out/alternative1";
            std::printf("Controller %d is registering callback for \"%s\"...\n", ownId, alternative1Topic.c_str());
            altSubscribers.emplace_back(new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, alternative1Topic, 1));
            altSubscribers.back()->registerCallback(&BMController::alternativeCallback, this);

            // Set up alternative2 subscriber.
            std::string alternative2Topic = rosName + "/out/alternative2";
            std::printf("Controller %d is registering callback for \"%s\"...\n", ownId, alternative2Topic.c_str());
            altSubscribers.emplace_back(new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, alternative2Topic, 1));
            altSubscribers.back()->registerCallback(&BMController::receiveAlternative, this);

            // Set up subscriber for matching.
            std::string matchingTopic = rosName + "/out/matchingResult";
            std::printf("Controller %d is registering callback for \"%s\"...\n", ownId, matchingTopic.c_str());
            matchingSubscribers.emplace_back(new message_filters::Subscriber<geometry_msgs::Point32>(*nh, matchingTopic, 1));
            matchingSubscribers.back()->registerCallback(&BMController::matchingCallback, this);
             */
        }

        // Regardless of whether or not this is own robot, subscribe to its location.
        std::string locationTopic = rosName + "/out/location";
        //std::printf("Controller %d is registering callback for \"%s\"...\n", ownId, locationTopic.c_str());
        locationSubscribers.emplace_back(new message_filters::Subscriber<geometry_msgs::Polygon>(*nh, locationTopic, 1));
        locationSubscribers.back()->registerCallback(&BMController::locationCallback, this);

    } // for each robot.
}

void BMController::locationCallback(const geometry_msgs::Polygon& msg)
{
    incReceiveCounter();
    updateLog();
    if (!(currentState == FOLLOWING_PATH || currentState == COORDINATING))
    {
        return; // We don't care about other robots' locations if we're not going anywhere.
    }
    int id = (int)(msg.points[0].x);
    auto loc = msg.points[1];
    PointD3D worldLoc(loc.x, loc.y, loc.z);
    Point3D gridLoc = grid.get()->getGridPoint(worldLoc);
    //std::cout << id << "'s location: [" << loc.x << ", " << loc.y << ", " << loc.z << "] = [" <<
    //          gridLoc.getX() << ", " << gridLoc.getY() << ", " << gridLoc.getZ() << "]" << std::endl;

    // Update our knowledge of that robot's location.
    //std::lock_guard<std::mutex> lock(robotLocations_mutex); // change this to using a timed mutex?
    robotLocations_mutex.lock();
    bool locked = true;
    if (robotLocations[id].isUninitialized())
    {
        // Set our knowledge of this robot's location for the first time.
        robotLocations[id] = gridLoc;
        robotLocations_mutex.unlock();
        locked = false;
        return;
    }
    if (robotLocations[id] != gridLoc) // Check if this constitutes a change in grid position.
    {
        // Update our knowledge of this robot's location.
        robotLocations[id] = gridLoc;
        robotLocations_mutex.unlock();
        locked = false;
        robotLocationChanged(id); // check if coordination is necessary, and do so if it is.
    }
    // We call robotLocationChanged only when a grid position has changed for any known robot (including self).
    // This dramatically cuts down the calls to this function which tests for minimum safe distance.
    if (locked)
        robotLocations_mutex.unlock();
}

BMController::~BMController()
{
    std::cout << "Destroying controller.\n";
    stopDriving();
    if (log != nullptr)
        log->closeLog(myID);
    for (auto& sub : locationSubscribers) {
        //sub.shutdown();
        sub->unsubscribe();
    }
/*    for (auto& sub : altSubscribers) {
        //sub.shutdown();
        sub->unsubscribe();
    }*/
}

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
    WriteLog("Navigating to (%d, %d)...\n", row, col);
    setState(FOLLOWING_PATH);

    // Plan the path.
    Point3D myGridLoc = grid->getGridPoint(driver->getLocation());
    robotLocations[myID] = myGridLoc;
    dstar.init(myGridLoc.getX(), myGridLoc.getY(), row, col);
    dstar.replan();
    list<state> path = dstar.getPath();
    list<Point3D>* waypoints = fromState(path);
    statsGuard.lock();
    initialPathLength = (int)waypoints->size();
    statsGuard.unlock();

    // Stop current driving activity, if any.
    stopDriving();

    // Begin following this path.
    driver->enableMovement();

    //driveThread = new std::thread(this->driveAlong, waypoints); // this doesn't compile.
    driveThread = new std::thread(std::bind(&BMController::driveAlong, this, *waypoints));
    currentPath = waypoints;

    //driveAlong(waypoints); // old synchronous version

    //std::printf("Robot %d:\tNo longer following initially planned path.\n", myID);
}

// Drives along a list of grid points until interrupted.
void BMController::driveAlong(list<Point3D> waypoints)
{
    WriteLog("Beginning to follow path: ");
    for (auto iter = waypoints.begin(); iter != waypoints.end(); ++iter)
    {
        Point3D current = *iter;
        std::printf("(%d, %d) ", current.getX(), current.getY());
    }

    std::printf("\n");
    bool started = false;
    Point3D myloc = robotLocations[myID];
    for (auto iter = waypoints.begin(); iter != waypoints.end(); ++iter)
    {
        Point3D current = *iter;
        if (!started)
        {
            if (myloc == current) // Ensure we start driving only to a place we've never been before.
            {
                started = true;
            }  else {
                WriteLog("We're already past (%d, %d)\n", current.getX(), current.getY());
            }
            continue;
        }

        WriteLog("Driving to (grid): (%d, %d)\n", current.getX(), current.getY());
        if (!driver->driveTo(grid->getWorldPoint(current)))
        {
            WriteLog("Driver was interrupted!\n");
            return;
        }

        // Increment distance travelled.
        ++totalDistanceTravelled;
        WriteLog("My total distance travelled is %d\n", totalDistanceTravelled);
    }

    if (!started)
    {
        WriteLog("I never found my own location (%d, %d) anywhere in the path!\n", myloc.getX(), myloc.getY());
    }

    WriteLog("Finished following path.\n");
    setState(REACHED_GOAL);
}

// make each controller subscribe to every other robot's out/location topic
// this will necessitate the messages from VREP being changed to include an ID from each robot
// (since they will all proc the same callback).

// when a robot is within minimum safe distance, enter coordinating state.

void BMController::robotLocationChanged(int id)
{
    if (currentState != FOLLOWING_PATH)
        return;

    robotLocations_mutex.lock();

    Point3D myGridLoc;
    bool startCoordinating = false;
    bool goalIsolation = false; // True if we need to avoid a stationary robot.

    // Update my own position.
    myGridLoc = grid.get()->getGridPoint(driver->getLocation()); // was *driver->myLoc.get()
    robotLocations[myID] = myGridLoc;

    WriteLog("I'm now at (%d, %d).\n", myGridLoc.getX(), myGridLoc.getY());

    // Recompute distances to every other robot.
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        if (i == myID)
            continue; // Don't check distance to myself.

        if (currentState == COORDINATING)
        {
            if (std::find(coordinatingWith.begin(), coordinatingWith.end(), i) != coordinatingWith.end())
            {
                WriteLog("Ignoring location update from Robot %d because we're already coordinating.\n",
                            i);
                continue; // Skip because we're already coordinating with this robot.
            }
            __glibcxx_assert(coordinatingWith.empty());
        }

        // Check distance
        Point3D gridDifference = myGridLoc - robotLocations[i];
        int cellDistance = gridDifference.manhattanNorm();
        //std::printf("Robot %d:\tI have moved to within %d cell of Robot %d.\n", myID, cellDistance, i);
        if (cellDistance <= MINIMUM_SAFE_DISTANCE)
        {
            WriteLog("Robot %d is within MSD. (%d cells away)\n", i, cellDistance);
            // React to nearby robot.
            // Instead of returning immediately, we keep going and mark all robots we need to coordinate with.

            if (checkIsCoordinating(i)) // Don't coordinate with nearby robot if it's inactive.
            {
                startCoordinating = true;
                robotAlternatives[i][0] = Alternative(Point3D(), -1);

                robotMatchings[i] = Matching(i);
                robotMatchings[i].setCost(-1);

                coordinatingWith.emplace_back(i);
                /*std::printf("Robot %d:\tcoordinatingWith.size() has grown to size %d: (%s)\n",
                            myID,
                            (int)coordinatingWith.size(),
                            vectorToString(coordinatingWith).c_str());
                */
            } else {
                WriteLog("I will not coordinate with robot %d because it is inactive.\n", i);

                // Mark other robot's location as impassable with D*.
                dstar.updateCell(robotLocations[i].getX(), robotLocations[i].getY(), NONTRAVERSABLE_COST);
                // Set flag to indicate we will have to replan.
                goalIsolation = true;
            }
            // this special value Alternative(0, -1) indicates we're coordinating with that robot and haven't received its alt yet.
        }
    } // for each robot.

    if (goalIsolation)
    {
        // We marked some cells as nontraversable because of inactive robots.
        // Now replan our path to avoid these cells.
        stopDriving();

        dstar.updateStart(myGridLoc.getX(), myGridLoc.getY());
        bool replanRet = dstar.replan();
        WriteLog("Replanned for goal isolation. Return value = %s\n", replanRet ? "ok" : "FAIL");
        list<state> path = dstar.getPath();
        list<Point3D>* waypoints = fromState(path);
        currentPath = waypoints;
    }

    robotLocations_mutex.unlock();

    if (startCoordinating)
    {
        // React to nearby robot.
        WriteLog("I'm about to start coordinating with %d other robots.\n", (int)coordinatingWith.size());
        setState(COORDINATING);
    } else {
        if (goalIsolation)
        {
            // Just avoid motionless obstacles.
            driver->enableMovement();
            driveThread = new std::thread(std::bind(&BMController::driveAlong, this, *currentPath));
        }
    }
}

void BMController::setState(State newState)
{
    switch (currentState)
    {
        case NOT_STARTED:
            switch (newState)
            {
                case NOT_STARTED:
                    WriteLog("No state change: Not Started --> Not Started\n");
                    break;
                case FOLLOWING_PATH:
                    WriteLog("CHANGING STATE: Not Started --> Following Path\n\n");
                    currentState = newState;

                    // Mark ourselves as able to coordinate.
                    enableCoordination();
                    break;
                case COORDINATING:
                    WriteLog("Invalid state transition: Not Started --> Coordinating\n");
                    break;
                case REACHED_GOAL:
                    WriteLog("Invalid state change: Not Started --> Reached Goal\n");
                    break;
            }
            break;
        case REACHED_GOAL:
            switch (newState)
            {
                case REACHED_GOAL:
                    WriteLog("No state change: Reached Goal --> Reached Goal\n");
                    break;
                case FOLLOWING_PATH:
                    WriteLog("Invalid state transition: Reached Goal --> Following Path\n");
                    break;
                case COORDINATING:
                    WriteLog("Invalid state transition: Reached Goal --> Coordinating\n");
                    break;
            }
            break;
        case FOLLOWING_PATH:
            switch (newState)
            {
                case REACHED_GOAL:
                    WriteLog("CHANGING STATE: Following Path --> Reached Goal\n\n");
                    currentState = newState;
                    // stop motors.
                    driver->disableMovement();
                    WriteLog("Stopping driving.\n");
                    stopDriving();

                    // Publish that we are not longer up for coordination.
                    disableCoordination();

                    // Signal to main thread that we're done.
                    activeWorkers->signal();

                    break;
                case FOLLOWING_PATH:
                    WriteLog("No state change: Following Path --> Following Path\n");
                    break;
                case COORDINATING:
                    WriteLog("CHANGING STATE: Following Path --> Coordinating\n\n");
                    currentState = newState;

                    // Stop moving.
                    stopDriving();

                    // Find (D*) and publish my alternate paths.
                    findAlternatePaths();

                    // Wait until I've received everyone else's alternatives.
                    //printf("Robot %d:\tBeginning to wait for alternatives.\n", myID);
                    waitForAllAlternatives();

                    if (coordinatingWith.empty())
                    {
                        WriteLog("Resuming driving because coordination timed out.\n");
                        setState(FOLLOWING_PATH);
                    }
                    addRobotsCoordinatedCounter((int)coordinatingWith.size());
                    incCoordinationCounter();

                    // Do bipartite matching.
                    myMatching->clear();
                    computeMatching();

                    // Share the matching you have found (id, cardinality, weight)
                    transmitMatchingResult();

                    // wait for matching info from every coordinating robot.
                    waitForAllMatchings();

                    // Decide if my matching is the best by the priority-ordered criteria:
                    // 1. involving the most robots (the "maximum matching") (max on cardinality)
                    // 2. having the lowest weight
                    // 3. originating from the robot with lowest ID number.
                    // For n=2, these are all trivial.
                    int bestMatchingId = findBestMatchingID();

                    list<Point3D>* newPath;
                    // If mine is the best, transmit it.
                    if (bestMatchingId == myID)
                    {
                        WriteLog("MY MATCHING IS THE BEST. I will transmit it now.\n");
                        transmitFullMatching();
                        newPath = getPathFromFirstPoint(myMatching->getPlaceFor(myID));
                    } else {
                        // If mine is not the best, listen for the full matching result from the robot that has the best matching.
                        WriteLog("My matching is not the best. Waiting for full matching from %d...\n", bestMatchingId);
                        Point3D whereToGo = awaitFullMatching(bestMatchingId);
                        if (whereToGo.getZ() == -1)
                        {
                            WriteLog("I was absent from the best matching.\n");
                            newPath = nullptr;
                        } else {
                            newPath = getPathFromFirstPoint(whereToGo);
                        }
                    }
                    if (!newPath) // don't move for a certain time interval.
                    {
                        const int WAIT_SECONDS = 10;
                        WriteLog("Waiting for %d seconds....\n", WAIT_SECONDS);
                        disableCoordination();
                        std::this_thread::sleep_for(std::chrono::seconds(WAIT_SECONDS));
                        setState(FOLLOWING_PATH);
                    } else {
                        // Change state back to FOLLOWING_PATH and go to the location prescribed in the best matching.

                        currentPath = newPath;
                        WriteLog("Resuming driving after coordination.\n");
                        setState(FOLLOWING_PATH);
                    }

                    break;
            }
            break;
        case COORDINATING:
            switch (newState)
            {
                case REACHED_GOAL:
                    //std::cout << "Invalid state transition: Coordinating --> Reached Goal\n";
                    WriteLog("CHANGING STATE: Coordinating --> Reached Goal\n\n");
                    currentState = newState;
                    // stop motors.
                    stopDriving();

                    // Publish that we are not longer up for coordination.
                    disableCoordination();

                    // Signal to main thread that we're done.
                    activeWorkers->signal();
                    break;
                case FOLLOWING_PATH:
                    WriteLog("CHANGING STATE: Coordinating --> Following Path\n\n");
                    currentState = newState;
                    coordinatingWith.clear(); // Clear the list of robots I'm coordinating with.
                    enableCoordination();
                    // todo: publish NULL alternatives and matchings here? in case anyone is erroneously coordinating with us
                    // Start following our path.
                    driver->enableMovement();
                    driveThread = new std::thread(std::bind(&BMController::driveAlong, this, *currentPath));
                    break;
                case COORDINATING:
                    WriteLog("No state change: Coordinating --> Coordinating\n");
                    break;
            }
            break;
    }
}

static Point3D getSecond(list<state> path) // This works.
{
    auto iter = path.begin();
    iter++;
    state pt = *iter;
    return Point3D(pt.x, pt.y, 0);
}

// Returns the alternative path that begins with the given point.
list<Point3D>* BMController::getPathFromFirstPoint(Point3D pt)
{
    if (pt == robotLocations[myID])
    {
        // We chose the stay-still alternative. Return null to indicate this.
        WriteLog("The chosen matching says I should stay still at (%d, %d).\n", pt.getX(), pt.getY());
        return nullptr;
    }

    Point3D altStart = getSecond(myAlternative1);
    //std::printf("Robot %d:\tSecond point in my 1st alternative path is (%d, %d).\n", myid, altStart.getX(), altStart.getY());

    list<Point3D>* ret;
    if (pt == altStart)
    {
        WriteLog("(%d, %d) == (%d, %d), indicating my 1st alternative.\n",
        pt.getX(), pt.getY(), altStart.getX(), altStart.getY());
        ret = fromState(myAlternative1);
        //ret->pop_front();
        return ret;
    } else {
        altStart = getSecond(myAlternative2);
        //std::printf("Robot %d:\tSecond point in my 2nd alternative path is (%d, %d).\n", myid, altStart.getX(), altStart.getY());
        if (pt == altStart)
        {
            WriteLog("(%d, %d) == (%d, %d), indicating my 2nd alternative.\n",
                        pt.getX(), pt.getY(), altStart.getX(), altStart.getY());
            ret = fromState(myAlternative2);
            //ret->pop_front();
            return ret;
        } else {
            // it's hitting this with value from old matching! my matching not getting cleared/recalculated or something the 2nd time 'round?

            WriteLog("(%d, %d) does not match either of my alternatives!\n",
                        pt.getX(), pt.getY());
            // I was prescribed to go to a location that does not match either of my alternatives!
            //__glibcxx_assert(false);
            return nullptr;
        }
    }
}

// Returns the point this robot should go to according to the matching published by the robot with the specified ID.
// This method blocks until an answer has been received.
// If this robot is not present in the matching, a Point3D with z=-1 is returned.
Point3D BMController::awaitFullMatching(int sourceId) // new method, not added to class definition yet.
{
    __glibcxx_assert(sourceId != myID); // I shouldn't be trying to receive matching from myself!

    // Build the name of the ROS topic that will publish the full matching.
    std::string topicName(baseName);
    topicName.append('_' + std::to_string(sourceId) + "/out/fullMatching");

    // Receive the full matching.
    auto msg = ros::topic::waitForMessage<geometry_msgs::Polygon>(topicName);
    incReceiveCounter();

    // Search for my own ID in the matching.
    auto pts = msg->points;
    for (auto iter = pts.begin(); iter != pts.end(); ++iter)
    {
        geometry_msgs::Point32 current = *iter;
        int x = (int)current.x;
        int y = (int)current.y;
        int z = (int)current.z;
        WriteLog("Robot %d says robot %d should go to (%d, %d)\n", sourceId, z, x, y);
        if (z == myID)
        {
            return Point3D(x, y, 0);
        }
    }
    // We failed to find our own ID among any of those in the matching.
    return Point3D(-1, -1, -1);
}

void BMController::transmitFullMatching()
{
    geometry_msgs::Polygon msg;
    for (auto iter = coordinatingWith.begin(); iter != coordinatingWith.end(); ++iter)
    {
        int id = *iter;
        geometry_msgs::Point32 pt;
        pt.z = id;
        if (myMatching->hasPlaceFor(id))
        {
            Point3D place = myMatching->getPlaceFor(id);
            pt.x = place.getX();
            pt.y = place.getY();

        } else {
            pt.x = -1;
            pt.y = -1;
        }
        msg.points.push_back(pt);
    }
    fullMatchingPublisher.publish(msg);
}

int BMController::findBestMatchingID()
{
    // Decide the robot with the best matching according to the priority-ordered criteria:
    //      1. involving the most robots (the "maximum matching") (max on cardinality)
    //      2. having the lowest weight
    //      3. originating from the robot with lowest ID number.

    // Select the matchings with highest cardinality.
    coordinatingWith.emplace_back(myID); // Consider also myself here.

    vector<int> bestCardMatches;
    int maxCardinality = -1;
    for (auto iter = coordinatingWith.begin(); iter != coordinatingWith.end(); ++iter)
    {
        int otherId = *iter;
        Matching otherMatching = robotMatchings[otherId];
        int otherMatchingCard = otherMatching.getCardinality();
        if (otherMatchingCard > maxCardinality)
        {
            // This robot's matching is the new best so far.
            WriteLog("New best cardinality so far (%d): Robot %d\n", otherMatchingCard, otherId);
            maxCardinality = otherMatchingCard;
            bestCardMatches.clear();
            bestCardMatches.push_back(otherId);
        } else {
            if (otherMatchingCard == maxCardinality)
            {
                // This robot's matching ties for the best so far.
                WriteLog("Tie for best cardinality so far(%d): Robot %d\n", otherMatchingCard, otherId);
                bestCardMatches.push_back(otherId);
            }
        }
    }

    // Of these, select the matchings with lowest cost.
    vector<int> bestCostMatches;
    int minCost = INT_MAX;
    for (auto iter = bestCardMatches.begin(); iter != bestCardMatches.end(); ++iter)
    {
        int otherId = *iter;
        Matching otherMatching = robotMatchings[otherId];
        int otherMatchingCost = otherMatching.getCost();
        if (otherMatchingCost < minCost)
        {
            // This robot's matching is the new best so far.
            WriteLog("New best cost so far (%d): Robot %d\n", otherMatchingCost, otherId);
            minCost = otherMatchingCost;
            bestCostMatches.clear();
            bestCostMatches.push_back(otherId);
        } else {
            if (otherMatchingCost == minCost)
            {
                // This robot's matching ties for the best so far.
                WriteLog("Tie for best cost so far(%d): Robot %d\n", otherMatchingCost, otherId);
                bestCostMatches.push_back(otherId);
            }
        }
    }

    // Of these, select the matching with lowest ID.
    int ret = ROBOT_COUNT + 1; // at least as high as the maximum robot ID.
    for (auto iter = bestCostMatches.begin(); iter != bestCostMatches.end(); ++iter)
    {
        int otherId = *iter;
        if (ret > otherId)
        {
            ret = otherId;
        }
    }
    return ret;
}

void BMController::enableCoordination()
{
    std_msgs::Bool msg;
    msg.data = true;
    participatingPublisher.publish(msg);
    incSendCounter();
}

void BMController::disableCoordination()
{
    std_msgs::Bool msg;
    msg.data = false;
    participatingPublisher.publish(msg);
    incSendCounter();
}

bool BMController::checkIsCoordinating(int robotId)
{
    // build robot name from ID and baseName stored at construction time
    std::string otherName(baseName);
    otherName.append('_' + std::to_string(robotId));

    std::string topicName(otherName);
    topicName.append("/out/participating");
    // This should return instantly because the topic should be latched.
    auto msg = ros::topic::waitForMessage<std_msgs::Bool>(topicName);
    incReceiveCounter();
    return msg->data;
}

void BMController::computeMatching()
{
    BipartiteMatcher matcher(myID);
    auto myGridLoc = grid.get()->getGridPoint(driver->getLocation());

    matcher.addSelf(myID, myGridLoc.getX(), myGridLoc.getY(), totalDistanceTravelled);

    auto alt = robotAlternatives[myID][0];
    auto altLoc = alt.NextStep;
    matcher.addAlternative1(myID, altLoc.getX(), altLoc.getY(), alt.TotalCost);

    alt = robotAlternatives[myID][1];
    altLoc = alt.NextStep;
    matcher.addAlternative2(myID, altLoc.getX(), altLoc.getY(), alt.TotalCost);

    //std::printf("Robot %d:\tComputing matching involving %d other robots.\n", myID, (int)coordinatingWith.size());

    for (auto iter = coordinatingWith.begin(); iter != coordinatingWith.end(); ++iter)
    {
        int id = *iter;
        Alternative a1 = robotAlternatives[id][0];
        matcher.addAlternative1(id, // false indicates the robot is non-self. for n > 2 robots, this should become an ID.
                                a1.NextStep.getX(),
                                a1.NextStep.getY(),
                                a1.TotalCost
        );
        Alternative a2 = robotAlternatives[id][1];
        matcher.addAlternative2(id,
                                a2.NextStep.getX(),
                                a2.NextStep.getY(),
                                a2.TotalCost
        );
    }
    matcher.displayWeights();
    matcher.solve();

    Point res = matcher.getResult(myID);
    WriteLog("My matching says I should go to (%d, %d).\n", res.X, res.Y);
    myMatching->add(myID, Point3D(res.X, res.Y, 0));
    // Store results in class-level data structure.
    for (auto iter = coordinatingWith.begin(); iter != coordinatingWith.end(); ++iter) {
        int id = *iter;
        if (matcher.hasResultFor(id))
        {
            res = matcher.getResult(id);
            WriteLog("My matching says %d should go to (%d, %d).\n", id, res.X, res.Y);
            myMatching->add(id, Point3D(res.X, res.Y, 0));
        }
    }

    // Store the total cost of the matching.
    myMatching->setCost(matcher.getTotalCost());

    robotMatchings[myID] = *myMatching;
}

void BMController::transmitMatchingResult()
{
    geometry_msgs::Point32 mResult;
    mResult.x = myID;
    mResult.y = myMatching->getCardinality();
    mResult.z = myMatching->getCost();

    matchingResultPublisher.publish(mResult);
    incSendCounter();
}

// Blocks until we've learned about matchings from all the robots within our D_safe.
void BMController::waitForAllMatchings()
{
    while (true)
    {
        bool done = true;
        for (auto iter = coordinatingWith.begin(); iter != coordinatingWith.end(); ++iter)
        {
            int id = *iter;
            Matching m = robotMatchings[id];
            if (m.getCost() == -1)
            {
                WriteLog("Waiting for matching info from robot %d...\n", id);
                //done = false;
                //break;
                std::string topicName(baseName);
                topicName.append('_' + std::to_string(id) + "/out/matchingResult");
                auto msg = ros::topic::waitForMessage<geometry_msgs::Point32>(topicName);
                incReceiveCounter();
                matchingCallback(*msg);
            }
        }
        if (done)
            return;
        sleep(1); // sleep 1 second before checking again.
    }
}


// Blocks until we've received alternatives from all the robots within our D_safe.
void BMController::waitForAllAlternatives()
{
    for (int i=0; i<coordinatingWith.size(); ++i)
    {
        int id = coordinatingWith.at(i);
        WriteLog("Waiting for alternative 1 from robot %d...\n", id);
        //done = false;
        //break;
        std::string topicName(baseName);
        topicName.append('_' + std::to_string(id) + "/out/alternative1");
        ros::Duration timeout(5);
        auto msg = ros::topic::waitForMessage<geometry_msgs::Polygon>(topicName, timeout);
        if (!msg)
        {
           WriteLog("Message was null!\n");
           // Stop attempting to coordinate with this robot.
            if (std::find(coordinatingWith.begin(), coordinatingWith.end(), id) != coordinatingWith.end())
            {
                coordinatingWith.erase(coordinatingWith.begin()+i);
            }
           continue;
        }
        incReceiveCounter();
        receiveAlternative(*msg);

        WriteLog("Waiting for alternative 2 from robot %d...\n", id);
        topicName = baseName;
        topicName.append('_' + std::to_string(id) + "/out/alternative2");
        msg = ros::topic::waitForMessage<geometry_msgs::Polygon>(topicName);
        incReceiveCounter();
        receiveAlternative(*msg);
    }
}

void BMController::findAlternatePaths()
{
    Point3D self = robotLocations[myID];
    dstar.updateStart(self.getX(), self.getY());

    // Print path before blocking cells for first alternative.
    WriteLog("Original path: ");
    printPath(dstar.getPath());

    // todo: change this so that only robots WITHIN MSD of us are blocked for D*.

    // First alternate path: Assume no other robots move.
    for (int id = 0; id < ROBOT_COUNT; ++id)
    {
        if (id != myID)
        {
            // Only mark nontraversable those robots which are near to us.
            Point3D other = robotLocations[id];
            if ((other - self).manhattanNorm() <= MINIMUM_SAFE_DISTANCE)
            {
                if (other != self) // Don't block it if it's our own location.
                {
                    WriteLog("Blocking cell (%d, %d)...\n", other.getX(), other.getY());
                    dstar.updateCell(other.getX(), other.getY(), NONTRAVERSABLE_COST);
                }
            } else {
                WriteLog("Not blocking robot %d's location (%d, %d) because it's far enough.\n", id, other.getX(), other.getY());
            }
        }
    }

    dstar.updateStart(self.getX(), self.getY());
    bool replanRet = dstar.replan();
    WriteLog("Replanned 1st alt path. Return value = %s\n", replanRet ? "ok" : "fail");

    // Broadcast the next point and total cost of this alternate path.
    list<state> path = dstar.getPath();
    WriteLog("Replanned path 1: ");
    printPath(path);

    // Store the path for my own use later, during matching.
    myAlternative1 = list<state>(path); // copy constructor (?)

    state nextStep = *std::next(path.begin(), 1); // Get the 2nd point in this path (it begins in our current location).

    geometry_msgs::Polygon alt1;
    geometry_msgs::Point32 a1;
    a1.x = nextStep.x;
    a1.y = nextStep.y;
    geometry_msgs::Point32 id1;
    id1.x = myID;
    id1.y = 1; // Indicates that this is the first of my alternative paths.
    id1.z = totalDistanceTravelled + getPathCost(path); // Total cost of this new path. See page 859 top left.
    alt1.points.push_back(id1);
    alt1.points.push_back(a1);
    //std::printf("Robot %d:\tPublishing my 1st alternative...\n", myID);
    alt1Publisher.publish(alt1);
    incSendCounter();

    // Store my own alternative along with others for when I do matching.
    robotAlternatives[myID][0] = Alternative(Point3D(nextStep.x, nextStep.y, 0),id1.z);

    // Second alternate path: Assume all robots could move anywhere.
    for (int i = 0; i < ROBOT_COUNT; ++i)
    {
        if (i != myID)
        {
            Point3D other = robotLocations[i];
            // Only block neighbors of nearby robots who are also coordinating.
            if ((other - self).manhattanNorm() <= MINIMUM_SAFE_DISTANCE && checkIsCoordinating(i))
            {
                for (const auto neighbor : grid->getNeighbors(other))
                {
                    if (neighbor != self)
                    {
                        dstar.updateCell(neighbor.getX(), neighbor.getY(), NONTRAVERSABLE_COST);
                    }
                }
            } else {
                WriteLog("Not blocking robot %d's location (%d, %d) because it's far enough OR that robot is not coordinating.\n",
                            i, other.getX(), other.getY());
            }
        }
    }

    replanRet = dstar.replan();
    WriteLog("Replanned 2nd alt path. Return value = %s\n", replanRet ? "ok" : "fail");

    path = dstar.getPath();
    WriteLog("Replanned path 2: ");

    printPath(path);

    // Store the path for my own use later, during matching.
    myAlternative2 = list<state>(path); // copy constructor (?)

    // Broadcast the next point and total cost of this alternate path.
    nextStep = *std::next(path.begin(), 1); // Get the 2nd point in this path (it begins in our current location).

    geometry_msgs::Polygon alt2;
    geometry_msgs::Point32 a2;
    a2.x = nextStep.x;
    a2.y = nextStep.y;
    geometry_msgs::Point32 id2;
    id2.x = myID;
    id2.y = 2; // Indicates that this is the second of my alternative paths.
    id2.z = totalDistanceTravelled + getPathCost(path); // Total cost of this new path. See page 859 top left.
    alt2.points.push_back(id2);
    alt2.points.push_back(a2);
    //std::printf("Robot %d:\tPublishing my 2nd alternative...\n", myID);
    alt2Publisher.publish(alt2);
    incSendCounter();

    // Store my own alternative along with others for when I do matching.
    robotAlternatives[myID][1] = Alternative(Point3D(nextStep.x, nextStep.y, 0),id2.z);

    // Clear all the cells we just marked as nontraversable.
    // note: be careful do not clear the walls. or maybe go ahead and clear them, then call constructor methods again.
    int maxX = grid->getRowCount() - 1;
    int maxY = grid->getColumnCount() - 1;
    for (int id = 0; id < ROBOT_COUNT; ++id)
    {
        if (id != myID)
        {
            Point3D other = robotLocations[id];
            dstar.updateCell(other.getX(), other.getY(), DEFAULT_COST);
            for (const auto neighbor : grid->getNeighbors(other))
            {
                int nx = neighbor.getX();
                int ny =  neighbor.getY();
                if (nx > 0 && nx < maxX && ny > 0 && ny < maxY)
                    dstar.updateCell(nx, ny, DEFAULT_COST);
            }
        }
    }

    // Set walls again, in case we just cleared any.
    // setupWalls();
    // No longer necessary because we take care not to clear any wall cells in the above loop.
}


static int getPathCost(list<state>& path)
{
    return (int)path.size();
}

void BMController::matchingCallback(const geometry_msgs::Point32& msg)
{
    incReceiveCounter();
    if (currentState != COORDINATING)
    {
        return; // We don't care about matching messages if we aren't coordinating.
    }

    // Extract message fields.
    int id = (int)(msg.x);
    int cardinality = (int)(msg.y);
    int cost = (int)(msg.z);

    Matching newMatching(id);
    newMatching.setCost(cost);
    newMatching.setCardinality(cardinality);
    robotMatchings[id] = newMatching;
}

void BMController::receiveAlternative(const geometry_msgs::Polygon &msg)
{
    incReceiveCounter();
    if (!(currentState == FOLLOWING_PATH || currentState == COORDINATING))
    {
        return; // We don't care about other robots' alternatives if we're not going anywhere.
    }

    // Extract the message fields.
    int id = (int)(msg.points[0].x);
    int altNumber = (int)(msg.points[0].y);
    int cost = (int)(msg.points[0].z);
    auto loc = msg.points[1];
    Point3D gridLoc(loc.x, loc.y, loc.z);
    std::cout << "Robot " << myID << ":\tReceived robot " << id << "'s alternative #" << altNumber << ": [" <<
              loc.x << ", " << loc.y << ", " << loc.z << "]\n";

    Alternative alt;
    alt.NextStep = gridLoc;
    alt.TotalCost = cost;

    // Update our table of alternatives.
    robotAlternatives[id][altNumber - 1] = alt;
}

void BMController::stopDriving()
{
    if (driver)
        driver->disableMovement();
    /*
      if (driveThread && driveThread->joinable())
    {
        auto dtid = driveThread->get_id();
        auto id = this_thread::get_id();
        if (dtid != id)
            driveThread->join();
    }
     */
}

void BMController::updateLog() {
    if (log != nullptr)
    {
        bool haveStart = currentPath != nullptr;
        Point3D start, goal;
        if (haveStart) {
            start = currentPath->front();
            goal = currentPath->back();
        }

        double timeElapsed; // todo: calculate me using timeStarted

        statsGuard.lock();
        log->writeLog(myID,
                      start,
                      goal,
                      initialPathLength,
                      haveStart ? (int)currentPath->size() : 0,
                      timeElapsed,
                      messagesSent,
                      messagesReceived,
                      timesCoordinated,
                      robotsCoordinatedWith);

        statsGuard.unlock();
    }
}

void BMController::incReceiveCounter()
{
    statsGuard.lock();
    ++messagesReceived;
    statsGuard.unlock();
}
void BMController::incSendCounter()
{
    statsGuard.lock();
    ++messagesSent;
    statsGuard.unlock();
}
void BMController::incCoordinationCounter()
{
    statsGuard.lock();
    ++timesCoordinated;
    statsGuard.unlock();
}
void BMController::addRobotsCoordinatedCounter(int robots)
{
    statsGuard.lock();
    robotsCoordinatedWith +=  robots;
    statsGuard.unlock();
}

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

static list<Point3D>* fromState(list<state> s)
{
    auto ret = new list<Point3D>();
    for (auto iter = s.begin(); iter != s.end(); ++iter)
    {
        state current = *iter;
        Point3D p(current.x, current.y, 0);
        ret->push_back(p);
    }
    return ret;
}

static void printPath(list<state> path)
{
    for (auto iter = path.begin(); iter != path.end(); ++iter)
    {
        state current = *iter;
        std::printf("(%d, %d) ", current.x, current.y);
    }
    std::printf("\n");
}

static std::string vectorToString(const vector<int> v)
{
    std::string ret;
    for (auto iter = v.begin(); iter != v.end(); ++iter)
    {
        int current = *iter;
        ret.append(to_string(current) + ", ");
    }
    return ret;
}

void BMController::WriteLog(const char* fmt...)
{
    std::stringstream s;
    s <<  std::this_thread::get_id();
    std::string tid = s.str();
    std::string lsd = tid.substr(tid.size() - 4);

    consoleMutex.lock();
    printf("Robot %d\tThread %s:\t", myID, lsd.c_str());
    va_list args;
    va_start (args, fmt);
    vprintf (fmt, args);
    va_end (args);
    consoleMutex.unlock();
}
