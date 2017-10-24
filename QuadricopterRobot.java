import java.util.ArrayList;
import java.util.List;

import coppelia.FloatW;
import coppelia.FloatWA;
import coppelia.remoteApi;

public class QuadricopterRobot implements IVrepRobot {

	String name;

	remoteApi api;
	int clientID;
	int handle;
	int targetHandle; // handle to navigation target placeholder object.
	public QuadricopterRobot(remoteApi vrep, int clientID, String name, int handle, int targetHandle)
	{
		api = vrep;
		this.clientID = clientID;
		this.name = name;
		this.handle = handle;
		this.targetHandle = targetHandle;
		cacheParameters();
	}

	public QuadricopterRobot(VrepUtil vu, String name, int handle, int targetHandle)
	{
		api = vu.getAPI();
		clientID = vu.getClientID();

		this.name = name;
		this.handle = handle;
		this.targetHandle = targetHandle;
		cacheParameters();
	}

	float length, width;
	private void cacheParameters()
	{
		int result = 0;
		FloatW maxx = new FloatW(0);
		result |= api.simxGetObjectFloatParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_max_x, maxx, remoteApi.simx_opmode_blocking);
		FloatW minx = new FloatW(0);
		result |= api.simxGetObjectFloatParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_min_x, minx, remoteApi.simx_opmode_blocking);
		width = maxx.getValue() - minx.getValue();

		FloatW maxy = new FloatW(0);
		result |= api.simxGetObjectFloatParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_max_y, maxy, remoteApi.simx_opmode_blocking);
		FloatW miny = new FloatW(0);
		result |= api.simxGetObjectFloatParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_min_y, miny, remoteApi.simx_opmode_blocking);
		length = maxy.getValue() - miny.getValue();

		if (result != 0)
			throw new RuntimeException("Could not get basic information about the robot. Error: " + VrepUtil.decodeReturnCode(result));
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public int getHandle() {
		return handle;
	}

	@Override
	public float getWidth() {
		return width;
	}

	@Override
	public float getLength() {
		return length;
	}

	@Override
	public int turnLeft(float speed) {
		return 0;
	}

	@Override
	public int turnRight(float speed) {
		return 0;
	}

	@Override
	public int faceNorth() {
		return 0;
	}

	@Override
	public int faceSouth() {
		return 0;
	}

	@Override
	public int faceEast() {
		return 0;
	}

	@Override
	public int faceWest() {
		return 0;
	}

	@Override
	public int faceDirection(float angle) {
		return 0;
	}

	@Override
	public int facePoint(PointF3D point) {
		return 0;
	}

	@Override
	public int driveTo(PointF3D goal, float maxError) {
		Thread driveThread = beginDriveTo(goal, maxError);
		try {
			driveThread.join();
		}
		catch (InterruptedException ie)
		{
			driveThread.interrupt();
			stop();
			return 1;
		} finally {
			isDriving = false;
		}
		return 0;
	}

	class Driver implements Runnable
	{
		List<PointF3D> waypoints;
		float maxError;
		public Driver(PointF3D goal, float maxError)
		{
			waypoints = new ArrayList<>();
			waypoints.add(goal);
			this.maxError = maxError;
		}

		public Driver(List<PointF3D> waypoints, float maxError)
		{
			this.waypoints = waypoints;
			this.maxError = maxError;
		}

		public void run()
		{
			for (PointF3D wp : waypoints)
			{
				//System.out.format("Driving to %s...\n", goal.toString());
				FloatWA pt = new FloatWA(3);
				pt.getArray()[0] = wp.getX();
				pt.getArray()[1] = wp.getY();
				pt.getArray()[2] = wp.getZ();
				int ret = 0;
				double errDist;
				PointF3D myLocation;

				myLocation = getLocation();
				errDist = myLocation.getEuclidianDistance(wp);
				ret |= api.simxSetObjectPosition(clientID, targetHandle, -1, pt, remoteApi.simx_opmode_oneshot);
				// Block until we are within max_error of the goal.
				while (errDist > maxError)
				{
					myLocation = getLocation();
					errDist = myLocation.getEuclidianDistance(wp);
					//System.out.format("I am %.2fm away.\n", errDist);

					// todo: when multithreading this routine, call thread.yield() iff this robot's speed is below a certain amount.
					// the reason for this is that a robot needs urgent control iff its speed is high.
					Thread.yield();
				}
			}

			isDriving = false; // Update state in owning instance to signal completion.	
		}
	}

	boolean isDriving;
	Thread t;
	public Thread beginDriveTo(PointF3D goal, float maxError)
	{
		if (isDriving)
		{
			throw new RuntimeException("This Quadricopter is already driving somewhere!");
		}
		try {
			if (t != null)
				t.join(); // clean up old thread?
		} catch (InterruptedException e) { }
		t = new Thread(new Driver(goal, maxError));
		isDriving = true;
		t.start();
		return t;
	}

	public Thread beginDrivePath(List<PointF3D> waypoints, float maxError)
	{
		if (isDriving)
		{
			throw new RuntimeException("This Quadricopter is already driving somewhere!");
		}
		try {
			if (t != null)
				t.join(); // clean up old thread?
		} catch (InterruptedException e) { }
		t = new Thread(new Driver(waypoints, maxError));
		isDriving = true;
		t.start();
		return t;
	}

	@Override
	public int goForward() {
		return 0;
	}

	boolean isStreamingPosition = false;
	@Override
	public PointF3D getLocation()
	{
		FloatWA ret = new FloatWA(3);
		int result = 0;
		if (!isStreamingPosition)
		{
			result |= api.simxGetObjectPosition(clientID, handle, -1, ret, remoteApi.simx_opmode_streaming);
			isStreamingPosition = true;
		}
		if (result != remoteApi.simx_return_ok && result != remoteApi.simx_return_novalue_flag)
			throw new RuntimeException("Error starting to get position: " + VrepUtil.decodeReturnCode(result));
		do {
			result = api.simxGetObjectPosition(clientID, handle, -1, ret, remoteApi.simx_opmode_buffer);
			if (result == remoteApi.simx_return_novalue_flag)
			{
				result = api.simxGetObjectPosition(clientID, handle, -1, ret, remoteApi.simx_opmode_blocking); 
			}
		} while (result == remoteApi.simx_return_novalue_flag);

		if (result != 0 && result != remoteApi.simx_return_novalue_flag)
		{
			throw new RuntimeException("Failed to get position. Error: " + VrepUtil.decodeReturnCode(result));
		} else {
			float[] coords = ret.getArray();
			return new PointF3D(coords[0], coords[1], coords[2]);
		}
	}

	@Override
	public int stop() {
		if (isDriving)
		{
			t.interrupt();
		}
		assert (!t.isAlive());
		return driveTo(getLocation(), 0.5f * (length + width));
	}

	@Override
	public void say(String str) {
		api.simxAddStatusbarMessage(clientID, str, remoteApi.simx_opmode_blocking);

	}

	@Override
	public String toString()
	{
		return name;
	}

}
