public class Point3D
{
	private int x, y, z;
	public Point3D(int x, int y, int z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public int getX()
	{
		return x;
	}
	
	public int getY()
	{
		return y;
	}
	
	public int getZ()
	{
		return z;
	}
	
	@Override
	public String toString()
	{
		return String.format("(%d, %d, %d)", x, y, z);
	}
	
	public double getEuclidianDistance(Point3D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		double zdiff = z - other.z;
		return Math.sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff);
	}
	
	public double getManhattanDistance(Point3D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		double zdiff = z - other.z;
		return Math.abs(xdiff) + Math.abs(ydiff) + Math.abs(zdiff);
	}
	
	public double getChebyshevDistance(Point3D other)
	{
		double xdiff = Math.abs(x - other.x);
		double ydiff = Math.abs(y - other.y);
		double zdiff = Math.abs(z - other.z);

		// return max(xdiff, ydiff, zdiff).
		if (xdiff > ydiff)
		{
			if (xdiff > zdiff)
				return xdiff;
			return zdiff;
		}
		if (ydiff > zdiff)
			return ydiff;
		return zdiff;
	}
	
	@Override
	public boolean equals(Object o)
	{
		if (o instanceof Point3D)
		{
			Point3D other = (Point3D)o;
			return x == other.x && y == other.y && z == other.z;
		}
		if (o instanceof PointF3D)
		{
			PointF3D other = (PointF3D)o;
			return x == other.getX() && y == other.getY() && z == other.getZ();
		}
		return false;
	}
	
	@Override
	public int hashCode()
	{
		return (x << 20) | (y << 10) | z;
	}
}