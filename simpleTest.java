// Author: Geoff McQueen
// Date: 23 September 2017

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;

import coppelia.remoteApi;


public class simpleTest
{
	public static void main(String[] args)
	{
		//    	if (aStarTest())
		//    		return;
		if (testQuadricopter())
			return;
		//if (myTest())
		//return;
		if (testMultiRobot("Quadricopter", VrepRobotType.Air))
			return;
	}

	// Wrangles all robots whose names begin with the specified string. 
	static boolean testMultiRobot(String robotPrefix, VrepRobotType type)
	{
		remoteApi vrep = new remoteApi();
		vrep.simxFinish(-1);
		int clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 4000, 5);
		if (clientID == -1)
		{
			System.out.println("Could not connect to VREP.");
			return true;
		}
		VrepUtil vu = new VrepUtil(vrep, clientID);

		System.out.println("Getting robots...");

		List<IVrepRobot> bots = vu.getRobotsByName("Quadricopter", VrepRobotType.Air);

		int count = bots.size();
		System.out.format("Got %d robots.\n", count);

		vu.startSimulation();

		// Stop all robots in case they are moving.
		for (IVrepRobot bot : bots)
			bot.stop();

		Grid g = new Grid(
				9, // rows
				9, // columns
				4, // levels
				0.5f, //bot.getWidth(), // row spacing
				0.5f, //bot.getLength(), // column spacing
				0.25f, // level spacing.
				new PointF3D(-2, -2, 1), // origin
				6 // connectedness
				);

		// Send each robot to the nearest place on the grid.
		for (IVrepRobot bot : bots)
		{
			Point3D gridPoint = g.getGridCoordinates(bot.getLocation());
			PointF3D worldPoint = g.getWorldCoordinates(gridPoint);
			System.out.format("Telling %s to drive from %s to %s [grid %s].\n",
					bot.toString(),
					bot.getLocation().toString(),
					worldPoint.toString(),
					gridPoint.toString());
			bot.driveTo(worldPoint, 0.5f);

		}

		// Find the mapping from the set of robot positions to itself that
		// maximizes coordinate distance. To do this, just map every robot
		// go to the initial position of the most distant robot.
		Collection<Thread> jobs = new ArrayList<>(bots.size());
		for (IVrepRobot bot : bots)
		{
			Point3D myGridPos = g.getGridCoordinates(bot.getLocation());
			// Find most distant robot.
			double farthestDist = 0;
			IVrepRobot farthestRobot = bot; 
			for (IVrepRobot other : bots)
			{
				Point3D otherGridPos = g.getGridCoordinates(other.getLocation());
				double dist = g.getDistance(myGridPos, otherGridPos);
				if (dist > farthestDist)
				{
					farthestDist = dist;
					farthestRobot = other;
				}
			}
			AStar as = new AStar(g);
			List<Point3D> path = as.findPath(myGridPos, g.getGridCoordinates(farthestRobot.getLocation()));
			System.out.format("Driving %s...\n", bot.toString());
			jobs.add(bot.beginDrivePath(g.getWorldCoordinates(path), 0.5f));
			System.out.format("Done driving %s.\n", bot.toString());
		}
		try {
			for (Thread t : jobs)
				t.join();
		} catch (InterruptedException ie) {
			// We've been aborted. Try to stop all robots, then exit.
			for (IVrepRobot bot : bots)
			{
				bot.stop();
			}
		}
		
		vu.stopSimulation();
		vrep.simxFinish(clientID);
		System.out.println("Done.");
		return true;
	}

	static boolean aStarTest()
	{
		Grid g = new Grid(9, 9, 1, 10, 10, 0, new PointF3D(0, 0, 0), 4);
		AStar astar = new AStar(g);

		Point3D origin = new Point3D(0, 0, 0);
		Point3D destination = new Point3D(8, 8, 0);

		List<Point3D> path = astar.findPath(origin, destination);

		return true;
	}

	static boolean testQuadricopter()
	{
		remoteApi vrep = new remoteApi();
		vrep.simxFinish(-1);
		int clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 4000, 5);
		if (clientID == -1)
		{
			System.out.println("Could not connect to VREP.");
			return true;
		}
		VrepUtil vu = new VrepUtil(vrep, clientID);

		System.out.println("Getting robot.");

		IVrepRobot bot = new QuadricopterRobot(vu, "Quadricopter",
				vu.getObjectByName("Quadricopter"), 
				vu.getObjectByName("Quadricopter_target"));
		vu.startSimulation();
		vu.say("Stopping robot.");
		bot.stop();

		Grid g = new Grid(
				10, // rows
				10, // columns
				4, // levels
				0.5f, //bot.getWidth(), // row spacing
				0.5f, //bot.getLength(), // column spacing
				0.5f, // level spacing
				bot.getLocation(), // origin
				27 // connectedness
				);

		ConsoleDriver cd = new ConsoleDriver(g, bot);
		cd.drive();

		vu.stopSimulation();
		vrep.simxFinish(clientID);
		System.out.println("Done.");
		return true;
	}

	static boolean myTest()
	{
		remoteApi vrep = new remoteApi();
		vrep.simxFinish(-1);
		int clientID = vrep.simxStart("127.0.0.1", 19997, true, true, 4000, 5);
		if (clientID == -1)
		{
			System.out.println("Could not connect to VREP.");
			return true;
		}
		VrepUtil vu = new VrepUtil(vrep, clientID);

		System.out.println("Getting robot.");
		IVrepRobot bot = new PioneerRobot(vu, "Pioneer_p3dx",
				"Pioneer_p3dx_leftMotor",
				"Pioneer_p3dx_rightMotor");
		vu.startSimulation();
		vu.say("Stopping robot.");
		bot.stop();

		Grid g = new Grid(
				5, // rows
				5, // columns
				1, // levels
				0.5f, //bot.getWidth(), // row spacing
				0.5f, //bot.getLength(), // column spacing
				0, // level spacing
				bot.getLocation(), // origin
				8 // connectedness
				);

		ConsoleDriver cd = new ConsoleDriver(g, bot);
		cd.drive();

		vu.stopSimulation();
		vrep.simxFinish(clientID);
		System.out.println("Done.");
		return true;
	}

	static void doRandomWalk(IVrepRobot bot)
	{
		Random r = new Random();
		for (int i=1; i<20; ++i)
		{
			switch (r.nextInt(4))
			{
			case 0:
				bot.faceNorth();
				break;
			case 1:
				bot.faceSouth();
				break;
			case 2:
				bot.faceEast();
				break;
			case 3:
				bot.faceWest();
				break;
			}
			bot.goForward();
			try {Thread.sleep(500);} catch (InterruptedException ie) {}
		}
	}


}

