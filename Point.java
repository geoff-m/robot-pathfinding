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
	
	public static Point nearestTo(PointF point)
	{
		float oldx = point.getX();
		float oldy = point.getY();
		int newx = Math.round(oldx);
		int newy = Math.round(oldy);
		System.out.format("Rounded x=%.2f to x=%d and y=%.2f to y=%d.\n",
				oldx, newx, oldy, newy);
		return new Point(newx, newy);
	}
	
	public PointF toPointF()
	{
		return new PointF(x, y);
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