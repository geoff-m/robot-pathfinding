// Author: Geoff McQueen
// Date: 23 September 2017

public interface IVrepRobot {
	public String getName();
	public int getHandle();
	
	public float getWidth();
	public float getLength();
	
	public int turnLeft(float speed);
	public int turnRight(float speed);
	public int faceNorth();
	public int faceSouth();
	public int faceEast();
	public int faceWest();
	public int faceDirection(float angle);
	public int facePoint(PointF3D point);
	public int driveTo(PointF3D goal, float max_error);
	
	public int goForward();
	public PointF3D getLocation();
	
	public int stop();
	
	public void say(String str);
}
