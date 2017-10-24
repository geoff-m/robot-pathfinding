// Author: Geoff McQueen
// Date: 27 September 2017

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
		maxy = g.getColumnCount();
		grid = g;
	}

	public List<Point3D> findPath(Point3D origin, Point3D destination)
	{
		if (origin.equals(destination))
		{
			List<Point3D> ret = new ArrayList<>();
			ret.add(destination);
			return ret;
		}
		PriorityQueue<CostPoint> open = new PriorityQueue<>();
		HashMap<Point3D, CostPoint> closed = new HashMap<>();
		PriorityQueue<CostPoint> nextgen = new PriorityQueue<>();

		CostPoint newPoint = new CostPoint(origin, 0, null);
		open.add(newPoint);
		closed.put(origin, newPoint);
		boolean success = false;
		do {
			for (CostPoint cp : open)
			{
				Point3D myPoint = cp.getPoint();
				double myCost = cp.getCost();
				List<Point3D> myNeighbors = grid.getNeighbors(myPoint); 
				for (Point3D nextPoint : myNeighbors)
				{
					// Check to see if we've already processed this point.
					if (closed.containsKey(nextPoint))
					{
						CostPoint existingNeighbor = closed.get(nextPoint);
						// Update existing point with new cost, if it is lesser.
						if (existingNeighbor.getCost() > myCost)
						{
							//closed.put(nextPoint, new CostPoint(nextPoint, myCost + grid.getDistance(myPoint,  nextPoint), cp));
							closed.put(nextPoint, new CostPoint(nextPoint, myCost + myPoint.getEuclidianDistance(nextPoint), cp));
						}
					} else {
						// We've never seen this point before.
						//newPoint = new CostPoint(nextPoint, myCost + grid.getDistance(myPoint,  nextPoint), cp);
						newPoint = new CostPoint(nextPoint, myCost + myPoint.getEuclidianDistance(nextPoint), cp);
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
			Deque<Point3D> ret = new LinkedList<>();
			while (newPoint != null)
			{
				ret.addFirst(newPoint.getPoint());
				newPoint = newPoint.getFromPoint();
			}
			
			return (List<Point3D>)ret;
		} else {
			return Collections.<Point3D>emptyList();
		}

	}

}

class CostPoint implements Comparable<CostPoint>
{
	private Point3D p;
	private double cost;
	private CostPoint from;

	public CostPoint(Point3D p, double cost, CostPoint from)
	{
		this.p = p;
		this.cost = cost;
		this.from = from;
	}

	public Point3D getPoint()
	{
		return p;
	}

	public double getCost()
	{
		return cost;
	}
	
	public CostPoint getFromPoint()
	{
		return from;
	}

	@Override
	public int compareTo(CostPoint other) {
		return Double.compare(cost, other.cost);
	}

	@Override
	public String toString()
	{
		return String.format("%s (cost=%.2f)", p.toString(), cost);
	}

	@Override
	public int hashCode()
	{
		return p.hashCode();
	}
}
