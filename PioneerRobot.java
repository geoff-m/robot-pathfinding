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
	
	int length, width;
	private void cacheParameters()
	{
		int result = 0;
		IntW maxx = new IntW(0);
		result |= api.simxGetObjectIntParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_max_x, maxx, remoteApi.simx_opmode_blocking);
		IntW minx = new IntW(0);
		result |= api.simxGetObjectIntParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_min_x, minx, remoteApi.simx_opmode_blocking);
		width = maxx.getValue() - minx.getValue();
		
		IntW maxy = new IntW(0);
		result |= api.simxGetObjectIntParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_max_y, maxy, remoteApi.simx_opmode_blocking);
		IntW miny = new IntW(0);
		result |= api.simxGetObjectIntParameter(clientID, handle, remoteApi.sim_objfloatparam_objbbox_min_y, miny, remoteApi.simx_opmode_blocking);
		length = maxy.getValue() - miny.getValue();
		
		if (result != 0)
			throw new RuntimeException("Could not get basic information about the robot.");
	}
	
	@Override
	public int getLength()
	{
		return length;
	}
	
	@Override
	public int getWidth()
	{
		return width;
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
	public int goForward() {
		int ret = 0;
		ret |= api.simxSetJointTargetVelocity(clientID, leftMotor, 1.0f, remoteApi.simx_opmode_oneshot);
		ret |= api.simxSetJointTargetVelocity(clientID, rightMotor, 1.0f, remoteApi.simx_opmode_blocking);
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
	final float ANGLE_EPSILON = 0.01f;
	final float ANGLE_NORTH = 0;
	final float ANGLE_EAST = (float)Math.PI / 2;
	final float ANGLE_SOUTH = (float)Math.PI;
	final float ANGLE_WEST = 3 * (float) Math.PI / 2;
	
	@Override
	public int faceDirection(float angle)
	{
		float minerror = Float.MAX_VALUE;		// for debug.
		float maxerror = Float.MIN_VALUE;		// for debug.
		float minabserror = 0;		// for debug.
		
		float dir, error;
		float lastError = 0;
		int ret = 0;
		int COUNTER = 0;
		while (Math.abs(error = VrepUtil.angleBetween(dir = getHeading(), angle)) > ANGLE_EPSILON)
		{
			/*if (error < minerror)
				minerror = error;
			if (error > maxerror)
				maxerror = error;
			if (Math.abs(error) < minabserror)
				minabserror = error;
			
			float dError = error - lastError;
			ret |= turnLeft(
					dError * dError * 1f +
					  error * 1f );

			
			if (++COUNTER % 100 == 0)
				System.out.format("Error from %.4f: %.4f.\n", angle, error);
			
			lastError = error;
			*/
			
			if (error < 0)
				turnLeft(1);
			else
				turnRight(1);

			// Tight loop: check heading again as soon as possible.
		}
		stop();
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


}
