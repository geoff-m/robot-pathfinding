// A zero-based 2D grid of points.
public class Grid
{
	int rows, cols;
	float row_spacing, col_spacing;
	float originX, originY;
	
	public Grid(int rows, int columns, 
			float row_spacing, float column_spacing, 
			PointF origin)
	{
		this.rows = rows;
		cols = columns;
		this.row_spacing = row_spacing;
		this.col_spacing = column_spacing;
		originX = origin.getX();
		originY = origin.getY();
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
}
