public class Point
{
	private int x, y;
	public Point(int x, int y)
	{
		this.x = x;
		this.y = y;
	}

	public int getX()
	{
		return x;
	}
	
	public int getY()
	{
		return y;
	}
	
	@Override
	public String toString()
	{
		return String.format("(%d, %d)", x, y);
	}
	
	public double getEuclidianDistance(Point other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.sqrt(xdiff * xdiff + ydiff * ydiff);
	}
	
	public double getManhattanDistance(Point other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.abs(xdiff) + Math.abs(ydiff);
	}
	
	@Override
	public boolean equals(Object o)
	{
		if (o instanceof Point)
		{
			Point other = (Point)o;
			return x == other.x && y == other.y;
		}
		return false;
	}
}