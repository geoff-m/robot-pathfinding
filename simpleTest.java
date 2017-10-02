// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

import java.util.Random;

import coppelia.IntW;
import coppelia.IntWA;
import coppelia.remoteApi;
// Make sure to have the server side running in V-REP: 
// in a child script of a V-REP scene, add following command
// to be executed just once, at simulation start:
//
// simExtRemoteApiStart(19999)
//
// then start simulation, and run this program.
//
// IMPORTANT: for each successful call to simxStart, there
// should be a corresponding call to simxFinish at the end!

public class simpleTest
{
    public static void main(String[] args)
    {
    	if (myTest())
    		return;
        System.out.println("Program started");
        remoteApi vrep = new remoteApi();
        vrep.simxFinish(-1); // just in case, close all opened connections
        int clientID = vrep.simxStart("127.0.0.1",19997,true,true,5000,5);
        if (clientID!=-1)
        {
            System.out.println("Connected to remote API server");   

            // Now try to retrieve data in a blocking fashion (i.e. a service call):
            IntWA objectHandles = new IntWA(1);
            int ret=vrep.simxGetObjects(clientID,vrep.sim_handle_all,objectHandles,vrep.simx_opmode_blocking);
            if (ret==vrep.simx_return_ok)
                System.out.format("Number of objects in the scene: %d\n",objectHandles.getArray().length);
            else
                System.out.format("Remote API function call returned with error code: %d\n",ret);
                
            try
            {
                Thread.sleep(2000);
            }
            catch(InterruptedException ex)
            {
                Thread.currentThread().interrupt();
            }
    
            // Now retrieve streaming data (i.e. in a non-blocking fashion):
            long startTime=System.currentTimeMillis();
            IntW mouseX = new IntW(0);
            vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,mouseX,vrep.simx_opmode_streaming); // Initialize streaming
            while (System.currentTimeMillis()-startTime < 5000)
            {
                ret=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,mouseX,vrep.simx_opmode_buffer); // Try to retrieve the streamed data
                if (ret==vrep.simx_return_ok) // After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                    System.out.format("Mouse position x: %d\n",mouseX.getValue()); // Mouse position x is actualized when the cursor is over V-REP's window
            }
            
            // Now send some data to V-REP in a non-blocking fashion:
            vrep.simxAddStatusbarMessage(clientID,"Hello V-REP!",vrep.simx_opmode_oneshot);

            // Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
            IntW pingTime = new IntW(0);
            vrep.simxGetPingTime(clientID,pingTime);

            // Now close the connection to V-REP:   
            vrep.simxFinish(clientID);
        }
        else
            System.out.println("Failed connecting to remote API server");
        System.out.println("Program ended");
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
    	vu.say("Stopping robot.");
    	bot.stop();
    	
    	vu.say("Facing north.");
    	bot.faceNorth();
    	
    	doRandomWalk(bot);
    	
    	Random r = new Random();
    	for (int i=1; i<20; ++i)
    	{
    		switch (r.nextInt(4))
        	{
        	case 0:
        		vu.say("facing north.");
        		bot.faceNorth();
        		break;
        	case 1:
        		vu.say("facing south.");
        		bot.faceSouth();
        		break;
        	case 2:
        		vu.say("facing east.");
        		bot.faceEast();
        		break;
        	case 3:
        		vu.say("facing west.");
        		bot.faceWest();
        		break;
        	}
        	try {Thread.sleep(1000);} catch (InterruptedException ie) {}	
    	}
    	
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
            
