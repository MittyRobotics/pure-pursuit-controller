package team1351.purepursuit.paths;


import team1351.purepursuit.TrajectoryPoint;
import team1351.purepursuit.Waypoint;

import java.awt.geom.Point2D;

/**
 * Bezier Curve Path Object.
 * <p>
 * Contains functions for generating a bezier curve based on a set of {@link Waypoint}.
 *
 * @author Owen Leather
 * @version 1.0
 */

public class BezierCurvePath {
	/** Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.*/
	private int steps;
	/** Array of {@link Waypoint}s that the curve passes through. */
	private Waypoint[] waypoints;

	/**
	 * Constructor.
	 *
	 * @param waypoints Array of {@link Waypoint}s that the curve passes through.
	 * @param steps Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.
	 */
	public BezierCurvePath(Waypoint[] waypoints, int steps) {
		this.steps = steps;
		this.waypoints = waypoints;
	}

	/**
	 * Generates the path.
	 *
	 * For each set of {@link Waypoint}s, generate a new segment of the path that passes through those waypoints. Add
	 * all of the segments together to get the final path, which can be generated based on any number of defining waypoints.
	 *
	 * @return Array of {@link TrajectoryPoint}s that make upm  the {@link BezierCurvePath}.
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
	 * Each segment of the path is the part of the path between each set of two {@link Waypoint}.
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
		if (firstSegment) {
			p0 = waypoint0.getWaypoint();
			p1 = waypoint0.getHandle();
			p2 = waypoint1.getHandle();
			p3 = waypoint1.getWaypoint();
		} else {
			p0 = waypoint0.getWaypoint();
			p1 = waypoint0.getOppositeHandle();
			p2 = waypoint1.getHandle();
			p3 = waypoint1.getWaypoint();
		}

		double t;
		for (int i = 0; i < steps; i++) {
			double a = 0;
			double b = steps-1;
			t = ((double)i - a) / (b - a);
			if(!lastSegment){
				t = Math.max(0, t - 0.01);
			}
			double x = Math.pow(1 - t, 3) * p0.getX() + 3 * Math.pow(1 - t, 2) * t * p1.getX() + 3 * (1 - t) * Math.pow(t, 2) * p2.getX() + Math.pow(t, 3) * p3.getX();
			double y = Math.pow(1 - t, 3) * p0.getY() + 3 * Math.pow(1 - t, 2) * t * p1.getY() + 3 * (1 - t) * Math.pow(t, 2) * p2.getY() + Math.pow(t, 3) * p3.getY();
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
		}
		return tradjectoryPoints;
	}
}
