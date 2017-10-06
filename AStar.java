import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

public class AStar {

	int maxx, maxy;
	Grid grid;
	public AStar(Grid g)
	{
		maxx = g.getRowCount();
		maxy = g.getColCount();
		grid = g;
	}

	public List<Point> findPath(Point origin, Point destination)
	{
		PriorityQueue<CostPoint> open = new PriorityQueue<>();
		HashMap<Point, CostPoint> closed = new HashMap<>();
		PriorityQueue<CostPoint> nextgen = new PriorityQueue<>();

		CostPoint newPoint = new CostPoint(origin, 0, null);
		open.add(newPoint);
		closed.put(origin, newPoint);
		boolean success = false;
		do {
			for (CostPoint cp : open)
			{
				Point myPoint = cp.getPoint();
				int myCost = cp.getCost();
				List<Point> myNeighbors = grid.getNeighbors(myPoint); 
				for (Point nextPoint : myNeighbors)
				{
					// Check to see if we've already processed this point.
					if (closed.containsKey(nextPoint))
					{
						CostPoint existingNeighbor = closed.get(nextPoint);
						// Update existing point with new cost, if it is lesser.
						if (existingNeighbor.getCost() > myCost)
						{
							closed.put(nextPoint, new CostPoint(nextPoint, myCost + 1, cp));
						}
					} else {
						// We've never seen this point before.
						newPoint = new CostPoint(nextPoint, myCost + 1, cp);
						closed.put(nextPoint, newPoint);
						if (nextPoint.equals(destination))
						{
							// We've discovered the destination.
							success = true;
							break;
						}
						nextgen.add(newPoint); // Explore the new point next time.
					}
				} // For each neighbor of the current point from 'open'.
				if (success)
					break;
			} // For each point in 'open'.
			
			open.clear();
			open.addAll(nextgen);
			nextgen.clear();
		} while (!open.isEmpty() && !success);
		
		if (success)
		{
			// Build the path from 'closed'.
			Deque<Point> ret = new LinkedList<>();
			while (newPoint != null) // It's inescapable
			{
				ret.addFirst(newPoint.getPoint());
				newPoint = newPoint.getFromPoint();
			}
			
			return (List<Point>)ret;
		} else {
			return Collections.<Point>emptyList();
		}

	}

}

class CostPoint implements Comparable<CostPoint>
{
	private Point p;
	private int cost;
	private CostPoint from;

	public CostPoint(Point p, int cost, CostPoint from)
	{
		this.p = p;
		this.cost = cost;
		this.from = from;
	}

	public Point getPoint()
	{
		return p;
	}

	public int getCost()
	{
		return cost;
	}
	
	public CostPoint getFromPoint()
	{
		return from;
	}

	@Override
	public int compareTo(CostPoint o) {
		return o.cost - cost;
	}

	@Override
	public String toString()
	{
		return String.format("%s (cost=%d)",p.toString(), cost);
	}

	@Override
	public int hashCode()
	{
		return (p.getX() << 14) | p.getY();
	}
}
