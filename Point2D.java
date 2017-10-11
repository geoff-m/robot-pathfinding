public class Point2D
{
	private int x, y;
	public Point2D(int x, int y)
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
	
	public double getEuclidianDistance(Point2D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.sqrt(xdiff * xdiff + ydiff * ydiff);
	}
	
	public double getManhattanDistance(Point2D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.abs(xdiff) + Math.abs(ydiff);
	}
	
	public double getChebyshevDistance(Point2D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		return Math.max(Math.abs(xdiff), Math.abs(ydiff));
	}
	
	// This method ignores the 3rd coordinate.
	public static Point2D nearestTo(PointF3D point)
	{
		float oldx = point.getX();
		float oldy = point.getY();
		int newx = Math.round(oldx);
		int newy = Math.round(oldy);
		//System.out.format("Rounded (%.3f, %.3f) to (%d, %d).\n", oldx, oldy, newx, newy);
		return new Point2D(newx, newy);
	}
	
	public PointF3D toPointF()
	{
		return new PointF3D(x, y, 0);
	}
	
	@Override
	public boolean equals(Object o)
	{
		if (o instanceof Point2D)
		{
			Point2D other = (Point2D)o;
			return x == other.x && y == other.y;
		}
		return false;
	}
}