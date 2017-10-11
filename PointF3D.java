public class PointF3D
{
	private float x, y, z;
	public PointF3D(float x, float y, float z)
	{
		this.x = x;
		this.y = y;
		this.z = z;
	}
	
	public float getX()
	{
		return x;
	}
	
	public float getY()
	{
		return y;
	}
	
	public float getZ()
	{
		return z;
	}
	
	@Override
	public String toString()
	{
		return String.format("(%.2f, %.2f, %.2f)", x, y, z);
	}
	
	public double getEuclidianDistance(PointF3D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		double zdiff = z - other.z;
		return Math.sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff);
	}
	
	public double getManhattanDistance(PointF3D other)
	{
		double xdiff = x - other.x;
		double ydiff = y - other.y;
		double zdiff = z - other.z;
		return Math.abs(xdiff) + Math.abs(ydiff) + Math.abs(zdiff);
	}
	
	public double getChebyshevDistance(PointF3D other)
	{
		float xdiff = x - other.x;
		float ydiff = y - other.y;
		float zdiff = z - other.z;

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
		if (o instanceof PointF3D)
		{
			PointF3D other = (PointF3D)o;
			return x == other.x && y == other.y && z == other.z;
		}
		return false;
	}
}