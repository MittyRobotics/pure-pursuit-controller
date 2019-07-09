package team1351.purepursuit.paths;


import team1351.purepursuit.TrajectoryPoint;
import team1351.purepursuit.Waypoint;

import java.awt.geom.Point2D;

/**
 * Cubic Hermite Spline Path Object.
 * <p>
 * Contains functions for generating a cubic hermite spline path based on a set of {@link Waypoint}s. Each waypoint
 * should contain a point and an angle.
 *
 * @author Owen Leather
 * @version 1.0
 */

public class CubicHermiteSplinePath {
	/** Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.*/
	private int steps;
	/** Array of {@link Waypoint}s that the curve passes through. */
	private Waypoint[] waypoints;

	/**
	 * Constructor.
	 *
	 * @param waypoints Array of {@link Waypoint}s that the path passes through.
	 * @param steps Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.
	 */
	public CubicHermiteSplinePath(Waypoint[] waypoints, int steps) {
		this.steps = steps;
		this.waypoints = waypoints;
	}

	/**
	 * Generates the path.
	 *
	 * For each set of {@link Waypoint}s, generate a new segment of the path that passes through those waypoints. Add
	 * all of the segments together to get the final path, which can be generated based on any number of defining waypoints.
	 *
	 * @return Array of {@link TrajectoryPoint}s that make up the {@link CubicHermiteSplinePath}.
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

			TrajectoryPoint[] segment = generateSegment(waypoints[i], waypoints[i + 1], stepsPerSegment, i == 0);
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
	private TrajectoryPoint[] generateSegment(Waypoint waypoint0, Waypoint waypoint1, int steps, boolean firstSegment) {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		double x0,x1,y0,y1,a0,a1,d,mx0,mx1,my0,my1;
		x0 = waypoint0.getWaypoint().getX();
		x1 = waypoint1.getWaypoint().getX();
		y0 = waypoint0.getWaypoint().getY();
		y1 = waypoint1.getWaypoint().getY();

		a0 = Math.toRadians(waypoint0.getAngle());
		a1 = Math.toRadians(waypoint1.getAngle());

		d = waypoint0.getWaypoint().distance(waypoint1.getWaypoint());
		mx0 = Math.cos(a0) * d;
		my0 = Math.sin(a0) * d;
		mx1 = Math.cos(a1) * d;
		my1 = Math.sin(a1) * d;
		double t;
		for (int i = 0; i < steps; i++) {
			double a = 0;
			double b = steps;
			t = ((double)i - a) / (b - a);
			t = Math.max(0, t - 0.01);

			double h0,h1,h2,h3;

			h0 = 2*Math.pow(t,3)-3*Math.pow(t,2)+1;
			h1 = Math.pow(t,3)-2*Math.pow(t,2)+t;
			h2 = -2*Math.pow(t,3)+3*Math.pow(t,2);
			h3 = Math.pow(t,3)-Math.pow(t,2);

			double x = h0*x0+h1*mx0+h2*x1+h3*mx1;
			double y = h0*y0+h1*my0+h2*y1+h3*my1;
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
			System.out.println(x + " " + y + " " + t + " " + a0 + " " + a1 + " " + d);
		}

		return tradjectoryPoints;
	}
}
