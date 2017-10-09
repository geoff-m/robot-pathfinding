// Author: Geoff McQueen
// Date: 29 September 2017
import java.util.ArrayList;
import java.util.List;

// A zero-based 2D grid of points.
public class Grid
{
	int rows, cols;
	float row_spacing, col_spacing;
	float originX, originY;
	int connectedness;
	
	public Grid(int rows, int columns, 
			float row_spacing, float column_spacing, 
			PointF origin,
			int connectedness)
	{
		this.rows = rows;
		cols = columns;
		this.row_spacing = row_spacing;
		this.col_spacing = column_spacing;
		originX = origin.getX();
		originY = origin.getY();
		if (connectedness != 4 && connectedness != 8)
			throw new IllegalArgumentException("Only 4- and 8-connected grids are supported.");
		this.connectedness = connectedness;
	}
	
	// Returns the absolute world coordinates of the specified location on the grid.
	public PointF getWorldCoordinates(Point gridPoint)
	{
		int row = gridPoint.getX();
		int col = gridPoint.getY();
		if (row >= rows || row < 0 || col >= cols || col < 0)
			throw new RuntimeException("That point is not on the grid.");
		
		return new PointF(originX + row * row_spacing,
						 originY + col * col_spacing);
	}
	
	// Returns the grid point (e.g. "(5, 3)") that is nearest to the given world point.
	public Point getGridCoordinates(PointF worldPoint)
	{
		// Get the nearest row to world.x.
		int row = (int) (0.5 + (worldPoint.getX() - originX) / row_spacing);
		int col = (int) (0.5 + (worldPoint.getY() - originY) / col_spacing);
		
		if (row >= rows)
			row = rows - 1;
		if (col >= cols)
			col = cols - 1;
		
		return new Point(row, col);
	}
	
	public int getRowCount()
	{
		return rows;
	}
	
	public int getColCount()
	{
		return cols;
	}
	
	public List<Point> getNeighbors(Point p)
	{
		int x = p.getX();
		int y = p.getY();
		if (x >= rows || y >= cols || x < 0 || y < 0)
		{
			throw new RuntimeException("That point is not in the grid.");
		}

		ArrayList<Point> ret = new ArrayList<>(connectedness);
		boolean inTopRow = x == 0;
		boolean inBottomRow = x == rows - 1;
		boolean inLeftColumn = y == 0;
		boolean inRightColumn = y == cols - 1;
		
		if (!inTopRow)
		{
			ret.add(new Point(x-1, y));
		}
		if (!inBottomRow)
		{
			ret.add(new Point(x+1, y));
		}
		if (!inLeftColumn)
		{
			ret.add(new Point(x, y-1));
		}
		if (!inRightColumn)
		{
			ret.add(new Point(x, y+1));
		}
		if (connectedness == 8)
		{
			if (!inTopRow && !inLeftColumn)
				ret.add(new Point(x - 1, y - 1));
			if (!inTopRow && !inRightColumn)
				ret.add(new Point(x - 1, y + 1));
			if (!inBottomRow && !inLeftColumn)
				ret.add(new Point(x + 1, y - 1));
			if (!inBottomRow && !inRightColumn)
				ret.add(new Point(x + 1, y + 1));
		}
		return ret;
	}
	
	public double getDistance(Point x, Point y)
	{
		if (connectedness == 8)
			return x.getChebyshevDistance(y);
		return x.getManhattanDistance(y); // connectedness == 4.
	}
}