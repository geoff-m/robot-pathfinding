
public interface IVrepRobot {
	public String getName();
	public int getHandle();
	
	public int getWidth();
	public int getLength();
	
	public int turnLeft(float speed);
	public int turnRight(float speed);
	public int faceNorth();
	public int faceSouth();
	public int faceEast();
	public int faceWest();
	public int faceDirection(float angle);
	
	public int goForward();
	
	public int stop();
}
