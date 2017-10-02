public class PointF
{
	float x, y;
	public PointF(float x, float y)
	{
		this.x = x;
		this.y = y;
	}
	
	public float getX()
	{
		return x;
	}
	
	public float getY()
	{
		return y;
	}
	
	@Override
	public String toString()
	{
		return String.format("(%.2f, %.2f)", x, y);
	}
	
	public double getEuclidianDistance(PointF other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.sqrt(xdiff * xdiff + ydiff * ydiff);
	}
	
	public double getManhattanDistance(PointF other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.abs(xdiff) + Math.abs(ydiff);
	}
}