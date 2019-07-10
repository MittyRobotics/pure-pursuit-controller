package team1351.purepursuit.paths;

import team1351.purepursuit.TrajectoryPoint;
import team1351.purepursuit.Waypoint;

import java.awt.geom.Point2D;

/**
 * Linear Path Object.
 * <p>
 * Contains functions for generating a set of lines based on a set of {@link Waypoint}s to create a path.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class LinearPath {
	/** Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.*/
	private int steps;
	/** Array of {@link Waypoint}s that the curve passes through. */
	private Waypoint[] waypoints;


	/**
	 * Constructor
	 *
	 * @param waypoints Array of {@link Waypoint}s that are the endpoints of the lines.
	 * @param steps Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.
	 */
	public LinearPath(Waypoint[] waypoints, int steps) {
		this.steps = steps;
		this.waypoints = waypoints;
	}

	/**
	 * Generates the path.
	 *
	 * For each set of {@link Waypoint}s, generate a new segment of the path that passes through those waypoints. Add
	 * all of the segments together to get the final path, which can be generated based on any number of defining waypoints.
	 *
	 * @return Array of {@link TrajectoryPoint}s that make up the {@link LinearPath}.
	 */
	public TrajectoryPoint[] generate() {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		int prevSegmentLength = 0;
		int stepsPerSegment = steps/(waypoints.length-1);
		int addedSteps = 0;

		if(stepsPerSegment*(waypoints.length-1) < steps){
			addedSteps = (steps-(stepsPerSegment*(waypoints.length-1)));
		}

		for (int i = 0; i < waypoints.length - 1; i++) {
			if(i== waypoints.length-2){
				stepsPerSegment+=addedSteps;
			}

			TrajectoryPoint[] segment = generateSegment(waypoints[i], waypoints[i + 1], stepsPerSegment, i == 0, i == waypoints.length-2);
			for (int a = 0; a < segment.length; a++) {
				tradjectoryPoints[a +prevSegmentLength] = segment[a];
			}
			prevSegmentLength = segment.length+prevSegmentLength;
		}

		return tradjectoryPoints;
	}

	/**
	 * Generates a segment of the path.
	 *
	 * Each segment of the path is the part of the path between each set of two {@link Waypoint}. For the {@link LinearPath},
	 * each segment is a set of {@link TrajectoryPoint}s on a line between the two waypoints.
	 *
	 * @param waypoint0 First {@link Waypoint} to generate the segment.
	 * @param waypoint1 Second {@link Waypoint} to generate the segment.
	 * @param steps Amount of points in this segment.
	 * @param firstSegment If the segment is the first segment in a path.
	 * @return An array of {@link TrajectoryPoint}s that create a segment of the path.
	 */
	private TrajectoryPoint[] generateSegment(Waypoint waypoint0, Waypoint waypoint1, int steps, boolean firstSegment, boolean lastSegment) {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		Point2D p0, p1, p2, p3;
		p0 = waypoint0.getWaypoint();
		p1 = waypoint1.getWaypoint();
		double d = p0.distance(p1);
		double t;
		for (int i = 0; i < steps; i++) {
			double a = 0;
			double b = steps-1;
			t = ((double)i - a) / (b - a);
			if(!lastSegment){
				t = Math.max(0, t - 0.01);
			}

			double angle = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
			double x = p0.getX() + Math.cos(angle) * (d * t);
			double y = p0.getY() + Math.sin(angle) * (d * t);
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
		}

		return tradjectoryPoints;
	}
}
