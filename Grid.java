// Author: Geoff McQueen
// Date: 29 September 2017
import java.util.ArrayList;
import java.util.List;


// todo: make this a 3D grid. update neighbors function. (implement min-Z/max-Z, etc.)

// todo: remove these limitations:
// A zero-based 2D grid of points.
// The type PointF3D is used, but the grid always lies in the plane of the first two coordinate axes.
// All PointF3Ds returned from this grid will have z = origin.getZ().
public class Grid
{
	int rows, cols, lvls;
	float row_spacing, col_spacing, lvl_spacing;
	float originX, originY, originZ;
	int connectedness;

	public Grid(int rows, int columns, int levels,
			float row_spacing, float column_spacing, float level_spacing, 
			PointF3D origin,
			int connectedness)
	{
		this.rows = rows;
		this.cols = columns;
		this.lvls = levels;
		this.row_spacing = row_spacing;
		this.col_spacing = column_spacing;
		this.lvl_spacing = level_spacing;
		originX = origin.getX();
		originY = origin.getY();
		originZ = origin.getZ();

		if (rows == 0 || cols == 0 || lvls == 0)
		{
			// If grid has zero volume, allow 4- and 8-connectedness.
			if (connectedness != 4 && connectedness != 8)
				throw new IllegalArgumentException("For flat grids, only 4- and 8-connected graphs are supported.");

		} else {
			// If grid has nonzero volume, allow 6- and 27-connectedness.
			if (connectedness != 6 && connectedness != 27)
				throw new IllegalArgumentException("For space-occupying grids, only 6- and 27-connected graphs are supported.");
		}

		this.connectedness = connectedness;
	}

	// Returns the absolute world coordinates of the specified location on the grid.
	public PointF3D getWorldCoordinates(Point3D gridPoint)
	{
		int row = gridPoint.getX();
		int col = gridPoint.getY();
		int lvl = gridPoint.getZ();
		if (row >= rows || row < 0 || col >= cols || col < 0 || lvl >= lvls || lvl < 0)
			throw new RuntimeException("That point is not on the grid.");

		return new PointF3D(originX + row * row_spacing,
				originY + col * col_spacing,
				originZ + lvl * lvl_spacing);
	}

	public List<PointF3D> getWorldCoordinates(List<Point3D> gridPoints)
	{
		List<PointF3D> ret = new ArrayList<>(gridPoints.size());
		for (Point3D gridPoint : gridPoints)
		{
			int row = gridPoint.getX();
			int col = gridPoint.getY();
			int lvl = gridPoint.getZ();
			if (row >= rows || row < 0 || col >= cols || col < 0 || lvl >= lvls || lvl < 0)
				throw new RuntimeException("That point is not on the grid.");

			ret.add(new PointF3D(originX + row * row_spacing,
					originY + col * col_spacing,
					originZ + lvl * lvl_spacing));
		}
		return ret;
	}

	// Returns the grid point (e.g. "(5, 3)") that is nearest to the given world point.
	public Point3D getGridCoordinates(PointF3D worldPoint)
	{
		int row = (int) (0.5 + (worldPoint.getX() - originX) / row_spacing);
		int col = (int) (0.5 + (worldPoint.getY() - originY) / col_spacing);
		int lvl = (int) (0.5 + (worldPoint.getZ() - originZ) / lvl_spacing);

		row = clamp(0, row, rows - 1);
		col = clamp(0, col, cols - 1);
		lvl = clamp(0, lvl, lvls - 1);

		return new Point3D(row, col, lvl);
	}

	private static int clamp(int minimum, int value, int maximum)
	{
		if (value > maximum)
			value = maximum;
		if (value < minimum)
			return minimum;
		return value;
	}

	public int getRowCount()
	{
		return rows;
	}

	public int getColumnCount()
	{
		return cols;
	}

	public int getLevelCount()
	{
		return lvls;
	}

	public boolean contains(Point3D p)
	{
		int x = p.getX();
		int y = p.getY();
		int z = p.getZ();
		return x < rows && x >= 0 && y < cols && y >= 0 && z < lvls && z >= 0;
	}

	public List<Point3D> getNeighbors(Point3D p)
	{
		int x = p.getX();
		int y = p.getY();
		int z = p.getZ();
		if (x >= rows || x < 0 || y >= cols || y < 0 || z >= lvls || z < 0)
			throw new RuntimeException("That point is not on the grid.");

		ArrayList<Point3D> ret = new ArrayList<>(connectedness);
		boolean inFrontRow = x == 0;
		boolean inBackRow = x == rows - 1;
		boolean inLeftColumn = y == 0;
		boolean inRightColumn = y == cols - 1;
		boolean inBottomLevel = z == 0;
		boolean inTopLevel = z == lvls - 1;

		// Deal with nodes as 4-/6-connected. 
		if (!inFrontRow)
			ret.add(new Point3D(x - 1, y, z));
		if (!inBackRow)
			ret.add(new Point3D(x + 1, y, z));
		if (!inLeftColumn)
			ret.add(new Point3D(x, y - 1, z));
		if (!inRightColumn)
			ret.add(new Point3D(x, y + 1, z));
		if (!inBottomLevel)
			ret.add(new Point3D(x, y, z - 1));
		if (!inTopLevel)
			ret.add(new Point3D(x, y, z + 1));

		// Deal with nodes as 8-/27-connected.
		if (connectedness == 8 || connectedness == 27)
		{
			// Add neighbors in this level.
			if (!inFrontRow && !inLeftColumn)
				ret.add(new Point3D(x - 1, y - 1, z));
			if (!inFrontRow && !inRightColumn)
				ret.add(new Point3D(x - 1, y + 1, z));
			if (!inBackRow && !inLeftColumn)
				ret.add(new Point3D(x + 1, y - 1, z));
			if (!inBackRow && !inRightColumn)
				ret.add(new Point3D(x + 1, y + 1, z));

			// Add neighbors in level above.
			if (!inTopLevel)
			{
				ret.add(new Point3D(x, y, z + 1));

				if (!inFrontRow)
				{
					ret.add(new Point3D(x - 1, y, z + 1));
					if (!inLeftColumn)
						ret.add(new Point3D(x - 1, y - 1, z + 1));
					if (!inRightColumn)
						ret.add(new Point3D(x - 1, y + 1, z + 1));
				}

				if (!inBackRow)
				{
					ret.add(new Point3D(x + 1, y, z + 1));
					if (!inLeftColumn)
						ret.add(new Point3D(x + 1, y - 1, z + 1));
					if (!inRightColumn)
						ret.add(new Point3D(x + 1, y + 1, z + 1));	
				}

				if (!inLeftColumn)
					ret.add(new Point3D(x, y - 1, z + 1));

				if (!inRightColumn)
					ret.add(new Point3D(x, y + 1, z + 1));
			}

			// Add neighbors in level below.
			if (!inBottomLevel)
			{
				ret.add(new Point3D(x, y, z - 1));

				if (!inFrontRow)
				{
					ret.add(new Point3D(x - 1, y, z - 1));
					if (!inLeftColumn)
						ret.add(new Point3D(x - 1, y - 1, z - 1));
					if (!inRightColumn)
						ret.add(new Point3D(x - 1, y + 1, z - 1));
				}

				if (!inBackRow)
				{
					ret.add(new Point3D(x + 1, y, z - 1));
					if (!inLeftColumn)
						ret.add(new Point3D(x + 1, y - 1, z - 1));
					if (!inRightColumn)
						ret.add(new Point3D(x + 1, y + 1, z - 1));	
				}

				if (!inLeftColumn)
					ret.add(new Point3D(x, y - 1, z - 1));

				if (!inRightColumn)
					ret.add(new Point3D(x, y + 1, z - 1));
			}
		}
		return ret;
	}

	public double getDistance(Point3D x, Point3D y)
	{
		switch (connectedness)
		{
			case 4:
			case 6:
				return x.getManhattanDistance(y);
			case 8:
			case 27:
				return x.getChebyshevDistance(y);
		}
		throw new RuntimeException(); // unreachable.
	}
}