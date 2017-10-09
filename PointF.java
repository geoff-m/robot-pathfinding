public class PointF
{
	private float x, y;
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
	
	public double getChebyshevDistance(PointF other)
	{
		float xdiff = x - other.x;
		float ydiff = y - other.y;
		return Math.max(Math.abs(xdiff), Math.abs(ydiff));
	}
	
	@Override
	public boolean equals(Object o)
	{
		if (o instanceof PointF)
		{
			PointF other = (PointF)o;
			return x == other.x && y == other.y;
		}
		return false;
	}
}