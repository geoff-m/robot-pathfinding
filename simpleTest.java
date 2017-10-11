// Author: Geoff McQueen
// Date: 23 September 2017

import java.util.List;
import java.util.Random;

import coppelia.remoteApi;


public class simpleTest
{
    public static void main(String[] args)
    {
    	//if (aStarTest())
    	//	return;
    	if (testQuadricopter())
    		return;
    	if (myTest())
    		return;
    }
    
    static boolean aStarTest()
    {
    	Grid g = new Grid(5, 5, 10, 10, new PointF3D(0, 0, 0), 4);
    	AStar astar = new AStar(g);
    	
    	Point2D origin = new Point2D(2, 1);
    	Point2D destination = new Point2D(3, 4);
    	
    	List<Point2D> path = astar.findPath(origin, destination);
    	
    	
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
    	
    	vu.say("Getting robot.");
    	
    	IVrepRobot bot = new QuadricopterRobot(vu, "Quadricopter",
    			vu.getObjectByName("Quadricopter"), 
    			vu.getObjectByName("Quadricopter_target"));
    	vu.startSimulation();
    	vu.say("Stopping robot.");
    	bot.stop();
    	
    	Grid g = new Grid(
    			5, // rows
    			5, // columns
    			0.5f, //bot.getWidth(), // row spacing
    			0.5f, //bot.getLength(), // column spacing
    			bot.getLocation(), // origin
    			4 // connectedness
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
    	
    	vu.say("Getting robot.");
    	IVrepRobot bot = new PioneerRobot(vu, "Pioneer_p3dx",
						    			   "Pioneer_p3dx_leftMotor",
						    			   "Pioneer_p3dx_rightMotor");
    	vu.startSimulation();
    	vu.say("Stopping robot.");
    	bot.stop();
    	
    	Grid g = new Grid(
    			5, // rows
    			5, // columns
    			0.5f, //bot.getWidth(), // row spacing
    			0.5f, //bot.getLength(), // column spacing
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
            
