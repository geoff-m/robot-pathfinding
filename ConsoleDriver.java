// Author: Geoff McQueen
// Date: 6 October 2017
import java.util.List;
import java.util.Scanner;

public class ConsoleDriver {
	private Grid g;
	private IVrepRobot bot;
	private AStar astar;
	public ConsoleDriver(Grid g, IVrepRobot bot)
	{
		this.g = g;
		this.bot = bot;
		astar = new AStar(g);
	}

	public void drive()
	{
		Scanner read = new Scanner(System.in);
		while (true)
		{
			System.out.print("Command: ");
			String[] tokens = read.nextLine().toLowerCase().split("\\s+");
			if (handleNorth(tokens))				
				continue;
			if (handleEast(tokens))	
				continue;
			if (handleSouth(tokens))
				continue;
			if (handleWest(tokens))				
				continue;
			if (handleForward(tokens))
				continue;
			if (handleGoTo(tokens))
				continue;
			if (handleQuit(tokens))
				break;
			System.out.println("Unrecognized command.");
		}
		read.close();
	}

	private boolean handleNorth(String[] tokens)
	{
		if (tokens.length == 1)
		{
			if (tokens[0].equals("n") || tokens[0].equals("north"))
			{
				bot.say("Facing north...");
				bot.faceNorth();
				bot.say("Done facing north.");
				return true;
			}
		}
		return false;
	}

	private boolean handleEast(String[] tokens)
	{
		if (tokens.length == 1)
		{
			if (tokens[0].equals("e") || tokens[0].equals("east"))
			{
				bot.say("Facing east...");
				bot.faceEast();
				bot.say("Done facing east.");
				return true;
			}
		}
		return false;
	}

	private boolean handleSouth(String[] tokens)
	{
		if (tokens.length == 1)
		{
			if (tokens[0].equals("s") || tokens[0].equals("south"))
			{
				bot.say("Facing south...");
				bot.faceSouth();
				bot.say("Done facing south.");
				return true;
			}
		}
		return false;
	}

	private boolean handleWest(String[] tokens)
	{
		if (tokens.length == 1)
		{
			if (tokens[0].equals("w") || tokens[0].equals("west"))
			{
				bot.say("Facing west...");
				bot.faceWest();
				bot.say("Done facing west.");
				return true;
			}
		}
		return false;
	}

	private boolean handleForward(String[] tokens)
	{
		if (tokens.length == 1)
		{
			if (tokens[0].equals("fd") || tokens[0].equals("forward"))
			{
				bot.say("Going forward 1sec...");
				bot.goForward();
				try {Thread.sleep(1000);} catch (InterruptedException ie) {}
				bot.say("Done going forward.");
				bot.stop();
				return true;
			}
		}
		return false;
	}

	private boolean handleGoTo(String[] tokens)
	{
		if (tokens.length == 3)
		{
			if (tokens[0].equals("goto"))
			{
				int goalRow = -1, goalCol = -1;
				int maxRow = g.getColCount();
				int maxCol = g.getRowCount();
				try {
					goalRow = Integer.parseInt(tokens[1].trim());
					goalCol = Integer.parseInt(tokens[2].trim());
				} catch (NumberFormatException fe) {
					System.out.format("Expected [0, %d], [0, %d] after goto\n", maxRow, maxCol);
					return true;
				}

				bot.say(String.format("Going to (%d, %d)...\n", goalRow, goalCol));
				
				float maxerr = Math.min(bot.getWidth(), bot.getLength());
				maxerr *= 0.5;
				
				PointF currentLocation = bot.getLocation();
				Point currentGridLocation = g.getGridCoordinates(currentLocation);
				List<Point> path = astar.findPath(currentGridLocation, new Point(goalRow, goalCol));
				StringBuilder sb = new StringBuilder("Path:");
				for (Point waypoint: path)
				{
					sb.append(" ");
					sb.append(waypoint.toString());
				}
				System.out.println(sb.toString());
					
				for (Point waypoint : path)
				{
					PointF worldCoords = g.getWorldCoordinates(waypoint);
					System.out.format("Going to grid=(%d, %d) world=(%.2f, %.2f)...\n",
							waypoint.getX(),
							waypoint.getY(),
							worldCoords.getX(),
							worldCoords.getY());
					bot.driveTo(worldCoords, maxerr);
					System.out.format("The robot is nearest to %s.\n",
							g.getGridCoordinates(bot.getLocation()));
				}
				bot.say(String.format("Done going to (%d, %d).\n", goalRow, goalCol));
				return true;
			}
		}
		return false;
	}
	
	private boolean handleQuit(String[] tokens)
	{
		if (tokens.length == 1)
		{
			if (tokens[0].equals("quit") || tokens[0].equals("exit"))
			{
				bot.stop();
				return true;
			}
		}
		return false;
	}
}
