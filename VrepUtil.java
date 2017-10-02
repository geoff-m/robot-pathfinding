import coppelia.IntW;
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
	 
	 // Returns the smallest angle (radians) between the two input angles.
	 public static float angleBetween(float x, float y)
	 {
		 float diff = y - x;
		 if (diff < -Math.PI/2)
			 diff += Math.PI * 2;
		 if (diff > Math.PI/2)
			 diff -= Math.PI * 2;
		 return diff;
	 }
}