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
	 * @return Array of {@link TrajectoryPoint}s that create the {@link LinearPath}.
	 */
	public TrajectoryPoint[] generate() {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		int prevSegmentLength = 0;
		for (int i = 0; i < waypoints.length - 1; i++) {

			TrajectoryPoint[] segment = generateSegment(waypoints[i], waypoints[i + 1], steps / (waypoints.length - 1), i == 0);
			for (int a = 0; a < segment.length; a++) {
				tradjectoryPoints[a +prevSegmentLength] = segment[a];
				prevSegmentLength = segment.length+prevSegmentLength;
			}
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
	private TrajectoryPoint[] generateSegment(Waypoint waypoint0, Waypoint waypoint1, int steps, boolean firstSegment) {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		Point2D p0, p1, p2, p3;
		if (firstSegment) {
			p0 = waypoint0.getWaypoint();
			p1 = waypoint0.getHandle();
			p2 = waypoint1.getWaypoint();
			p3 = waypoint1.getHandle();
		} else {
			p0 = waypoint0.getWaypoint();
			p1 = waypoint0.getOppositeHandle();
			p2 = waypoint1.getHandle();
			p3 = waypoint1.getWaypoint();
		}
		double d0 = p0.distance(p1);
		double d1 = p1.distance(p2);
		double d2 = p2.distance(p3);
		double dt = d0 + d1 + d2;

		int s0 = (int) ((d0 / dt) * steps);
		int s1 = (int) ((d1 / dt) * steps);
		int s2 = (int) ((d2 / dt) * steps);

		if (s0 + s1 + s2 > steps) {
			s2 = (steps - (s0 + s1));
		} else if (s0 + s1 + s2 < steps) {
			s2 = (steps - (s0 + s1));
		}

		double t;
		for (int i = 0; i < s0; i++) {
			t = (double) i / s0;
			double a = Math.atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
			double x = p0.getX() + Math.cos(a) * (d0 * t);
			double y = p0.getY() + Math.sin(a) * (d0 * t);
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
		}
		for (int i = s0; i < s1 + s0; i++) {
			t = (double) i / (s1 + s0);
			double a = Math.atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX());
			double x = p1.getX() + Math.cos(a) * (d1 * t);
			double y = p1.getY() + Math.sin(a) * (d1 * t);
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
		}
		for (int i = (s1 + s0); i < steps; i++) {
			t = (double) i / steps;
			double a = Math.atan2(p3.getY() - p2.getY(), p3.getX() - p2.getX());
			double x = p2.getX() + Math.cos(a) * (d2 * t);
			double y = p2.getY() + Math.sin(a) * (d2 * t);
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
		}
		return tradjectoryPoints;
	}
}
