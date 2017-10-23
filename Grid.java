// Author: Geoff McQueen
// Date: 29 September 2017
import java.util.ArrayList;
import java.util.List;


// todo: make this a 3D grid. update neighbors function. (implement min-Z/max-Z, etc.)

// A zero-based 2D grid of points.
// The type PointF3D is used, but the grid always lies in the plane of the first two coordinate axes.
// All PointF3Ds returned from this grid will have z = origin.getZ(). 
public class Grid
{
	int rows, cols;
	float row_spacing, col_spacing;
	float originX, originY, originZ;
	int connectedness;

	public Grid(int rows, int columns, 
			float row_spacing, float column_spacing, 
			PointF3D origin,
			int connectedness)
	{
		this.rows = rows;
		cols = columns;
		this.row_spacing = row_spacing;
		this.col_spacing = column_spacing;
		originX = origin.getX();
		originY = origin.getY();
		originZ = origin.getZ();
		if (connectedness != 4 && connectedness != 8)
			throw new IllegalArgumentException("Only 4- and 8-connected grids are supported.");
		this.connectedness = connectedness;
	}

	// Returns the absolute world coordinates of the specified location on the grid.
	public PointF3D getWorldCoordinates(Point2D gridPoint)
	{
		int row = gridPoint.getX();
		int col = gridPoint.getY();
		if (row >= rows || row < 0 || col >= cols || col < 0)
			throw new RuntimeException("That point is not on the grid.");

		return new PointF3D(originX + row * row_spacing,
				originY + col * col_spacing,
				originZ);
	}

	// Returns the grid point (e.g. "(5, 3)") that is nearest to the given world point.
	public Point2D getGridCoordinates(PointF3D worldPoint)
	{
		int row = (int) (0.5 + (worldPoint.getX() - originX) / row_spacing);
		int col = (int) (0.5 + (worldPoint.getY() - originY) / col_spacing);

		if (row >= rows)
			row = rows - 1;
		if (col >= cols)
			col = cols - 1;

		return new Point2D(row, col);
	}

	public int getRowCount()
	{
		return rows;
	}

	public int getColCount()
	{
		return cols;
	}

	public boolean contains(Point2D p)
	{
		int x = p.getX();
		int y = p.getY();
		return x < rows && y < cols && x >= 0 && y >= 0;
	}

	public List<Point2D> getNeighbors(Point2D p)
	{
		int x = p.getX();
		int y = p.getY();
		if (x >= rows || y >= cols || x < 0 || y < 0)
		{
			throw new RuntimeException("That point is not in the grid.");
		}

		ArrayList<Point2D> ret = new ArrayList<>(connectedness);
		boolean inTopRow = x == 0;
		boolean inBottomRow = x == rows - 1;
		boolean inLeftColumn = y == 0;
		boolean inRightColumn = y == cols - 1;

		if (!inTopRow)
		{
			ret.add(new Point2D(x-1, y));
		}
		if (!inBottomRow)
		{
			ret.add(new Point2D(x+1, y));
		}
		if (!inLeftColumn)
		{
			ret.add(new Point2D(x, y-1));
		}
		if (!inRightColumn)
		{
			ret.add(new Point2D(x, y+1));
		}
		if (connectedness == 8)
		{
			if (!inTopRow && !inLeftColumn)
				ret.add(new Point2D(x - 1, y - 1));
			if (!inTopRow && !inRightColumn)
				ret.add(new Point2D(x - 1, y + 1));
			if (!inBottomRow && !inLeftColumn)
				ret.add(new Point2D(x + 1, y - 1));
			if (!inBottomRow && !inRightColumn)
				ret.add(new Point2D(x + 1, y + 1));
		}
		return ret;
	}

	public double getDistance(Point2D x, Point2D y)
	{
		if (connectedness == 8)
			return x.getChebyshevDistance(y);
		return x.getManhattanDistance(y); // connectedness = 4.
	}
}