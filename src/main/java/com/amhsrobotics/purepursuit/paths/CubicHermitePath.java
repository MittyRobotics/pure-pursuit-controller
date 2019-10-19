package com.amhsrobotics.purepursuit.paths;

import com.amhsrobotics.purepursuit.VelocityConstraints;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;

public class CubicHermitePath extends Path {

	public CubicHermitePath(Coordinate[] coordinates, VelocityConstraints velocityConstraints){
		super(coordinates, velocityConstraints);
	}

	@Override
	public void generatePoints() {
		int steps = 200;
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		int prevSegmentLength = 0;
		int stepsPerSegment = steps/(getCoordinates().length-1);
		int addedSteps = 0;

		if(stepsPerSegment*(getCoordinates().length-1) < steps){
			addedSteps = (steps-(stepsPerSegment*(getCoordinates().length-1)));
		}

		for (int i = 0; i < getCoordinates().length - 1; i++) {
			if(i== getCoordinates().length-2){
				stepsPerSegment+=addedSteps;
			}

			TrajectoryPoint[] segment = generateSegment(getCoordinates()[i], getCoordinates()[i + 1], stepsPerSegment, i == 0, i == getCoordinates().length-2);
			for (int a = 0; a < segment.length; a++) {
				tradjectoryPoints[a +prevSegmentLength] = segment[a];
			}
			prevSegmentLength = segment.length+prevSegmentLength;
		}

		setTrajectoryPoints(tradjectoryPoints);
	}

	private TrajectoryPoint[] generateSegment(Coordinate coordinate, Coordinate coordinate1, int steps, boolean firstSegment, boolean lastSegment) {
		TrajectoryPoint[] tradjectoryPoints = new TrajectoryPoint[steps];
		double x0,x1,y0,y1,a0,a1,d,mx0,mx1,my0,my1;
		x0 = coordinate.getX();
		x1 = coordinate1.getX();
		y0 = coordinate.getY();
		y1 = coordinate1.getY();

		a0 = Math.toRadians(coordinate.getAngle());
		a1 = Math.toRadians(coordinate1.getAngle());

		d = coordinate.distance(coordinate1);
		mx0 = Math.cos(a0) * d;
		my0 = Math.sin(a0) * d;
		mx1 = Math.cos(a1) * d;
		my1 = Math.sin(a1) * d;
		double t;
		for (int i = 0; i < steps; i++) {
			double a = 0;
			double b = steps-1;
			t = ((double)i - a) / (b - a);
			if(!lastSegment){
				t = Math.max(0, t - 0.05);
			}

			double h0,h1,h2,h3;

			h0 = 2*Math.pow(t,3)-3*Math.pow(t,2)+1;
			h1 = Math.pow(t,3)-2*Math.pow(t,2)+t;
			h2 = -2*Math.pow(t,3)+3*Math.pow(t,2);
			h3 = Math.pow(t,3)-Math.pow(t,2);

			double x = h0*x0+h1*mx0+h2*x1+h3*mx1;
			double y = h0*y0+h1*my0+h2*y1+h3*my1;
			tradjectoryPoints[i] = new TrajectoryPoint(x, y);
		}

		return tradjectoryPoints;
	}
}
