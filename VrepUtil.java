// Author: Geoff McQueen
// Date: 23 September 2017

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import coppelia.FloatWA;
import coppelia.IntW;
import coppelia.IntWA;
import coppelia.StringWA;
import coppelia.remoteApi;

public class VrepUtil {

	remoteApi api;
	int clientID;
	public VrepUtil(remoteApi vrep, int clientID)
	{
		api = vrep;
		this.clientID = clientID;
	}
	public remoteApi getAPI()
	{ return api; }
	public int getClientID()
	{ return clientID; }
	
	public int startSimulation()
	{
		return api.simxStartSimulation(clientID, remoteApi.simx_opmode_blocking);
	}
	
	public int stopSimulation()
	{
		return api.simxStopSimulation(clientID, remoteApi.simx_opmode_blocking);
	}

	// TODO: I do not know if simx_opmode_buffer has the caching behavior I think it does.
	// If not, cache this method's return values in a dictionary instead.
	public int getObjectByName(String name)
	{
    	IntW handle = new IntW(-1);
    	// Attempt to get value from cache.
    	int result = api.simxGetObjectHandle(clientID, name, handle, remoteApi.simx_opmode_buffer);
    	if (result == remoteApi.simx_return_novalue_flag)
    	{
    		// Send command and wait for response.
    		result = api.simxGetObjectHandle(clientID, name, handle, remoteApi.simx_opmode_blocking);
    	}
    	if (result != 0)
    	{
    		throw new RuntimeException("Failed to get object. The API returned " + decodeReturnCode(result));
    	}
    	return handle.getValue();
	}
	
	public List<IVrepRobot> getRobotsByName(String namePrefix, VrepRobotType type)
	{
		List<IVrepRobot> ret = null;
		if (type == VrepRobotType.Air)
		{
			// get all shape handles.
			IntWA shapeHandles = new IntWA(1);
			int retcode = 0;
			IntWA intData = new IntWA(1);
			FloatWA floatData = new FloatWA(1);
			StringWA stringData = new StringWA(1);
			retcode |= api.simxGetObjectGroupData(clientID, remoteApi.sim_appobj_object_type, 0, shapeHandles, intData, floatData, stringData, remoteApi.simx_opmode_blocking);
			
			int[] handles = shapeHandles.getArray();
			String[] names = stringData.getArray();
			int count = handles.length;
			ret = new ArrayList<IVrepRobot>();
			Pattern robotNameRegex = Pattern.compile(namePrefix + "(#\\d*)?");
			for (int i=0; i<count; ++i)
			{
				int handle = handles[i];
				String name = names[i];
				//System.out.format("Got object %d with name \"%s\".\n", handle, name);
				Matcher m = robotNameRegex.matcher(name);
				if (m.matches())
				{
					String suffix = m.group(1);
					String targetName = namePrefix + "_target" + (suffix !=null ? m.group(1) : "");
					int targetHandle = handles[indexOf(names, targetName)];
					ret.add(new QuadricopterRobot(this, name, handle, targetHandle));
				}
			}
		}
		return ret;
	}
	
	
	private static <T> int indexOf(T[] array, T item)
	{
		int len = array.length;
		for (int i=0; i<len; ++i)
			if (array[i].equals(item))
				return i;
		return -1;
	}
	
	 public static String decodeReturnCode(int remoteAPIReturnCode)
	    {
	    	switch (remoteAPIReturnCode)
	    	{
		    	case remoteApi.simx_return_ok:
		    		return "Success";
		    	case remoteApi.simx_return_novalue_flag:
		    		return "Value not in buffer";
		    	case remoteApi.simx_return_timeout_flag:
		    		return "Timed out";
		    	case remoteApi.simx_return_illegal_opmode_flag:
		    		return "Illegal opmode";
		    	case remoteApi.simx_return_remote_error_flag:
		    		return "Remote error";
		    	case remoteApi.simx_return_split_progress_flag:
		    		return "Split command already in progress";
		    	case remoteApi.simx_return_local_error_flag:
		    		return "Local error";
		    	case remoteApi.simx_return_initialize_error_flag:
		    		return "Not yet initialized";
	    		default:
	    			return "Multiple flags set: " + Integer.toBinaryString(remoteAPIReturnCode);
	    	}
	    }
	 
	 public int say(String message)
	 {
		 return api.simxAddStatusbarMessage(clientID, message, remoteApi.simx_opmode_oneshot);
	 }
	 
	 // Returns the smallest angle between the two input angles. The result will be between -Pi/2 and Pi/2. 
	 public static float angleBetween(float x, float y)
	 {
		 double diff = y - x;
		 
		 diff = mod(diff, 2 * Math.PI);
		 
		 if (diff > Math.PI)
			 diff = diff - Math.PI * 2;
		 
		 if (diff < -Math.PI)
			 diff = diff + Math.PI * 2;
		 
		 return (float)diff;
	 }
	 
	 private static double mod(double x, double m)
	 {
		 while (x < 0)
		 	x += m;
		 return x % m;
	 }
	 
	 public static double radiansToDegrees(float radians)
	 { 
         double bearing = (radians - 1.5708) / Math.PI * 180.0;

         if (bearing < 0.0){
             bearing = 360 + bearing;
         }
         
         return (bearing-180) * -1.0;
    }

    /*checking the difference between current angle & destination angle*/
    public static double calcDegreeDifference(double currentBearingDegrees,double destinationBearingDegrees)
    {    
    	double diff = destinationBearingDegrees - currentBearingDegrees;

         if (diff>180.0){
             diff=diff-360.0;
         }

         if(diff<-180.0){
             diff=diff+360.0;
         }

         return diff;
    }
	 
}