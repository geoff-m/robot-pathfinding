// Author: Geoff McQueen
// Date: 23 September 2017
import coppelia.FloatW;
import coppelia.FloatWA;
import coppelia.IntW;
import coppelia.remoteApi;

public class PioneerRobot implements IVrepRobot {

	String name;
	int handle;
	int leftMotor, rightMotor;
	
	remoteApi api;
	int clientID;
	
	public PioneerRobot(remoteApi vrep, int clientID, String name, int handle, int leftMotorHandle, int rightMotorHandle)
	{
		api = vrep;
		this.clientID = clientID;
		
		this.name = name;
		this.handle = handle;
		leftMotor = leftMotorHandle;
		rightMotor = rightMotorHandle;
		cacheParameters();
	}
	
	public PioneerRobot(VrepUtil vu, String name, String leftMotorName, String rightMotorName)
	{
		api = vu.getAPI();
		clientID = vu.getClientID();
		
		this.name = name;
		handle = vu.getObjectByName(name);
		leftMotor = vu.getObjectByName(leftMotorName);
		rightMotor = vu.getObjectByName(rightMotorName);
		cacheParameters();
	}
	
	@Override
	public String getName() {
		return name;
	}

	@Override
	public int getHandle() {
		return handle;
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
	public float getLength()
	{
		return length;
	}
	
	@Override
	public float getWidth()
	{
		return width;
	}
	
	boolean isStreamingPosition = false;
	@Override
	public PointF getLocation()
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
			return new PointF(ret.getArray()[0], ret.getArray()[1]); // these may be the wrong indices, i don't know
		}
	}
	
	boolean isStreamingHeading = false;
	public float getHeading()
	{
		FloatWA ret = new FloatWA(3);
		int result = 0;
		if (!isStreamingHeading)
		{
			result |= api.simxGetObjectOrientation(clientID, handle, -1, ret, remoteApi.simx_opmode_streaming);
			isStreamingHeading = true;
		}
		if (result != remoteApi.simx_return_ok && result != remoteApi.simx_return_novalue_flag)
			throw new RuntimeException("Error starting to get heading: " + VrepUtil.decodeReturnCode(result));
		do {
			result = api.simxGetObjectOrientation(clientID, handle, -1, ret, remoteApi.simx_opmode_buffer);
			if (result == remoteApi.simx_return_novalue_flag)
			{
				result = api.simxGetObjectOrientation(clientID, handle, -1, ret, remoteApi.simx_opmode_blocking); 
			}
		} while (result == remoteApi.simx_return_novalue_flag);
		
		if (result != 0 && result != remoteApi.simx_return_novalue_flag)
		{
			throw new RuntimeException("Failed to get heading. Error: " + VrepUtil.decodeReturnCode(result));
		} else {
			return ret.getArray()[2];
		}
	}
	
	@Override
	public int facePoint(PointF point)
	{
		PointF loc = getLocation();
		float angle = (float)Math.atan2(point.getY() - loc.getY(), point.getX() - loc.getX());
		return faceDirection(angle);
	}
	
	@Override
	public int driveTo(PointF goal, float max_error)
	{
		//System.out.format("Driving to %s...\n", goal.toString());
		int ret = 0;
		double errDist;
		PointF myLocation;
		
		myLocation = getLocation();
		errDist = myLocation.getManhattanDistance(goal);
		while (errDist > max_error)
		{
			ret |= facePoint(goal);
			goForward(1);
			myLocation = getLocation();
			errDist = myLocation.getManhattanDistance(goal);
			//System.out.format("I am %.2fm away.\n", errDist);
		}
		stop();
		
		return ret;
	}

	@Override
	public int goForward() {
		return goForward(1.0f);
	}
	
	public int goForward(float speed)
	{
		int ret = 0;
		ret |= api.simxSetJointTargetVelocity(clientID, leftMotor, speed, remoteApi.simx_opmode_oneshot);
		ret |= api.simxSetJointTargetVelocity(clientID, rightMotor, speed, remoteApi.simx_opmode_blocking);
		return ret;
	}

	@Override
	public int stop()
	{
		int ret = 0;
		ret |= api.simxSetJointTargetVelocity(clientID, leftMotor, 0f, remoteApi.simx_opmode_oneshot);
		ret |= api.simxSetJointTargetVelocity(clientID, rightMotor, 0f, remoteApi.simx_opmode_blocking);
		return ret;
	}
	
	final float TURN_SPEED_RATIO = 0.1f;
	@Override
	public int turnLeft(float speed) {
		int ret = 0;
		ret |= api.simxSetJointTargetVelocity(clientID, leftMotor, TURN_SPEED_RATIO * speed, remoteApi.simx_opmode_oneshot);
		ret |= api.simxSetJointTargetVelocity(clientID, rightMotor, speed, remoteApi.simx_opmode_blocking);
		return ret;
	}

	@Override
	public int turnRight(float speed) {
		int ret = 0;
		ret |= api.simxSetJointTargetVelocity(clientID, leftMotor, speed, remoteApi.simx_opmode_oneshot);
		ret |= api.simxSetJointTargetVelocity(clientID, rightMotor, TURN_SPEED_RATIO * speed, remoteApi.simx_opmode_blocking);
		return ret;
	}

	// todo: change all these as necessary.
	final float ANGLE_EPSILON = 0.052f; // 0.052 rad ~= 3 deg.
	final float ANGLE_NORTH = (float)Math.PI / 2;
	final float ANGLE_EAST = 0;
	final float ANGLE_SOUTH =  3 * (float)Math.PI / 2;
	final float ANGLE_WEST = (float) Math.PI;
	
	@Override
	public int faceDirection(float angle)
	{
		//System.out.format("Starting turn towards %.3f from %.3f\n", angle, getHeading());
		
		double minerror = Double.MAX_VALUE;		// for debug.
		double maxerror = Double.MIN_VALUE;		// for debug.
		double minabserror = Double.MAX_VALUE;		// for debug.
		final double DEGREE_EPSILON = 3;
		double error;
		double lastError = 0;
		int ret = 0;
		double degHeading = VrepUtil.radiansToDegrees(getHeading());
		double degGoal = VrepUtil.radiansToDegrees(angle);
		while (Math.abs(error = VrepUtil.calcDegreeDifference(
				degHeading = VrepUtil.radiansToDegrees(getHeading()),
				degGoal)) > DEGREE_EPSILON)
		{
			// debug: break on sign change.
			if (lastError > 0 ^ error > 0)
				error *= 1; // break into debugger.
			if (error < minerror)
				minerror = error;
			if (error > maxerror)
				maxerror = error;
			if (Math.abs(error) < minabserror)
				minabserror = Math.abs(error);
			
			if (error < 0)
				turnLeft(2);
			else
				turnRight(2);

			
			lastError = error;
			// Tight loop: check heading again as soon as possible.
		}
		stop();
		//System.out.format("Stopping turn with current heading = %.3f\n", degHeading);
		return ret;
	}
	
	@Override
	public int faceNorth()
	{
		return faceDirection(ANGLE_NORTH);
	}
	
	@Override
	public int faceSouth()
	{
		return faceDirection(ANGLE_SOUTH);
	}
	
	@Override
	public int faceEast()
	{
		return faceDirection(ANGLE_EAST);
	}
	
	@Override
	public int faceWest()
	{
		return faceDirection(ANGLE_WEST);
	}

	@Override
	public void say(String str)
	{
		api.simxAddStatusbarMessage(clientID, str, remoteApi.simx_opmode_blocking);
	}
	
}
