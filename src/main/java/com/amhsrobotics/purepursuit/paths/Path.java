package com.amhsrobotics.purepursuit.paths;

import com.amhsrobotics.purepursuit.PathFollowerPosition;
import com.amhsrobotics.purepursuit.VelocityConstraints;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;

import java.awt.*;
import java.awt.geom.Point2D;

public class Path {

	/**Set of defining coordinates for the path*/
	private Coordinate[] coordinates;

	/**The velocity constraints of the path*/
	private VelocityConstraints velocityConstraints;

	/**The array of generated points that makeup the path*/
	private TrajectoryPoint[] trajectoryPoints;

	/**
	 * Number of points at the end of the graph that are reduced
	 */
	private double slowdownPoints;

	/**
	 * Number divided by the velocity at the slowdown points.
	 */
	private double slowdownFactor;

	private double kCurvature = 1;

	private CoordinateSystem GENERATOR_COORDINATE_SYSTEM = new CoordinateSystem(
			90,
			TurnSign.POSITIVE,
			VectorDirection.POSITIVE_Y,
			VectorDirection.NEGATIVE_X);

	/**
	 * Path constructor
	 */
	public Path(Coordinate[] coordinates, VelocityConstraints velocityConstraints){
		this(coordinates,velocityConstraints,0,0);
	}

	/**
	 * Path constructor
	 */
	public Path(Coordinate[] coordinates, VelocityConstraints velocityConstraints, double slowdownPoints, double slowdownFactor){

		for(int i = 0; i < coordinates.length; i++){
			coordinates[i] = CoordinateManager.getInstance().coordinateTransformation(coordinates[i], GENERATOR_COORDINATE_SYSTEM);
		}
		this.coordinates = coordinates;
		this.velocityConstraints = velocityConstraints;
		this.kCurvature = velocityConstraints.getkCurvature();
		this.slowdownPoints = slowdownPoints;
		this.slowdownFactor = slowdownFactor;

		generatePoints();
		calculatePositions();
		calculateCurvature();
		calculateVelocities();
		calculateAngles();
	}

	/**
	 * Generates the points of the path.
	 */
	public void generatePoints(){
	}

	/**
	 * Calculates the position of each point along the path.
	 */
	public void calculatePositions(){
		if(trajectoryPoints != null){
			trajectoryPoints[0].setPosition(0);
			for(int i = 1; i < trajectoryPoints.length; i++){
				trajectoryPoints[i].setPosition(trajectoryPoints[i-1].distance(trajectoryPoints[i]) + trajectoryPoints[i-1].getPosition());
			}
		}
	}

	/**
	 * Calculates the curvature at each point on the path
	 */
	public void calculateCurvature(){
		trajectoryPoints[0].setCurvature(0);
		trajectoryPoints[trajectoryPoints.length-1].setCurvature(0);
		for(int i = 1; i < trajectoryPoints.length-1; i++){
			double x1 = trajectoryPoints[i - 1].getX();
			double y1 = trajectoryPoints[i - 1].getY();
			double x2 = trajectoryPoints[i].getX();
			double y2 = trajectoryPoints[i].getY();
			double x3 = trajectoryPoints[i + 1].getX();
			double y3 = trajectoryPoints[i + 1].getY();

			if (x1 == x2) {
				//Avoid divide by 0 errors
				x2 += 0.00001;
			}

			double k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2)) / (x1 - x2);
			double k2 = (y1 - y2) / (x1 - x2);
			double b = 0.5 * (Math.pow(x2, 2) - 2 * x2 * k1 + Math.pow(y2, 2) - Math.pow(x3, 2) + 2 * x3 * k1 - Math.pow(y3, 2)) / (x3 * k2 - y3 + y2 - x2 * k2);
			double a = k1 - k2 * b;
			
			double sign = findSide(new Point2D.Double(a,b),new Point2D.Double(x1,y1), new Point2D.Double(x2,y2));
			
			double r = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(y1 - b, 2));
			double curvature = 1 / r * sign;

			if (Double.isNaN(r)) {
				curvature = 0;
			}
			trajectoryPoints[i].setCurvature(curvature);
		}
	}
	
	
	/**
	 * Finds which side of a line the point is on
	 *
	 * @param p  point
	 * @param p1 point 1 of the line
	 * @param p2 point 2 of the line
	 * @return a -1 or 1 value based on which side the point is on
	 */
	private double findSide(Point2D.Double p, Point2D.Double p1, Point2D.Double p2) {
		double x = p.getX();
		double y = p.getY();
		double x1 = p1.getX();
		double y1 = p1.getY();
		double x2 = p2.getX();
		double y2 = p2.getY();
		return -Math.signum((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1));
	}
	
	/**
	 * Calculates the base velocity of each point on the path
	 */
	public void calculateVelocities(){
		for (int i = trajectoryPoints.length - 1; i > -1; i--) {
			double maxVelocityWithCurvature = Math.max(Math.min(velocityConstraints.getMaxVelocity(), kCurvature / trajectoryPoints[i].getCurvature()),velocityConstraints.getMinVelocity());
			maxVelocityWithCurvature = 100000;
			if (i == trajectoryPoints.length - 1) {
				trajectoryPoints[i].setVelocity(velocityConstraints.getEndVelocity());
			} else {
				double distance = TrajectoryPoint.distance(trajectoryPoints[i + 1], trajectoryPoints[i]);
				double velocity;
				if(velocityConstraints.getMaxDeceleration() != 0){
					velocity = Math.min(maxVelocityWithCurvature, Math.sqrt(Math.pow(trajectoryPoints[i + 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxDeceleration() * distance));
				}
				else {
					velocity = Math.min(maxVelocityWithCurvature, Math.sqrt(Math.pow(trajectoryPoints[i + 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance));
				}

				trajectoryPoints[i].setVelocity(velocity);
			}
		}
		for (int i = 0; i < trajectoryPoints.length; i++) {
			if(i == 0){
				trajectoryPoints[i].setTime(0);
			}
			if (i <= 0) {
				trajectoryPoints[i].setVelocity(velocityConstraints.getStartVelocity());
			} else {
				double distance = TrajectoryPoint.distance(trajectoryPoints[i - 1], trajectoryPoints[i]);
				double velocity = Math.min(trajectoryPoints[i].getVelocity(), Math.sqrt(Math.pow(trajectoryPoints[i - 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance));
				//System.out.println( Math.min(trajectoryPoints[i].getVelocity(), Math.sqrt(Math.pow(trajectoryPoints[i - 1].getVelocity(), 2) + 2 * velocityConstraints.getMaxAcceleration() * distance)) + " " + trajectoryPoints[i-1].getVelocity() + " " + trajectoryPoints[i].getVelocity() + " "+ i);
				double time = distance/velocity;

				if(Double.isInfinite(time)){
					time = 0;
				}

				trajectoryPoints[i].setVelocity(velocity);
				trajectoryPoints[i].setTime(trajectoryPoints[i-1].getTime() + time);
			}
		}
		trajectoryPoints[0].setVelocity(trajectoryPoints[1].getVelocity());
		for(int i = trajectoryPoints.length-1; i > trajectoryPoints.length-1-slowdownPoints; i--){
			trajectoryPoints[i].setVelocity(Math.min(Math.max(velocityConstraints.getMinVelocity(),trajectoryPoints[i].getVelocity()/slowdownFactor),trajectoryPoints[i].getVelocity()));
		}
	}
	
	private void calculateAngles(){
		for(int i = 0; i < trajectoryPoints.length-1; i++){
			trajectoryPoints[i].setAngle(90+calculateAngleOfPoint(trajectoryPoints[i].getX(), trajectoryPoints[i].getY(), trajectoryPoints[i+1].getX(), trajectoryPoints[i+1].getY()));
		}
	}
	
	private double calculateAngleOfPoint(double x1, double y1, double x2, double y2){
		double x = x1 - x2;
		double y = y1 - y2;
		double angleToLookahead = Math.toDegrees(Math.atan2(y,x));
		return angleToLookahead;
	}

	public Coordinate[] getCoordinates() {
		return coordinates;
	}

	public void setCoordinates(Coordinate[] coordinates) {
		this.coordinates = coordinates;
	}

	public VelocityConstraints getVelocityConstraints() {
		return velocityConstraints;
	}

	public void setVelocityConstraints(VelocityConstraints velocityConstraints) {
		this.velocityConstraints = velocityConstraints;
	}

	public TrajectoryPoint[] getTrajectoryPoints() {
		return trajectoryPoints;
	}

	public void setTrajectoryPoints(TrajectoryPoint[] trajectoryPoints) {
		this.trajectoryPoints = trajectoryPoints;
	}
}
