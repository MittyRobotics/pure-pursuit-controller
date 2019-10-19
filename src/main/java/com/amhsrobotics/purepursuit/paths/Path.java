package com.amhsrobotics.purepursuit.paths;

import com.amhsrobotics.purepursuit.VelocityConstraints;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;

public class Path {

	/**Set of defining coordinates for the path*/
	private Coordinate[] coordinates;

	/**The velocity constraints of the path*/
	private VelocityConstraints velocityConstraints;

	/**The array of generated points that makeup the path*/
	private TrajectoryPoint[] trajectoryPoints;

	private double kCurvature = 0.8;

	private CoordinateSystem GENERATOR_COORDINATE_SYSTEM = new CoordinateSystem(
			90,
			TurnSign.POSITIVE,
			VectorDirection.POSITIVE_Y,
			VectorDirection.NEGATIVE_X);


	/**
	 * Path constructor
	 */
	public Path(Coordinate[] coordinates, VelocityConstraints velocityConstraints){

		for(int i = 0; i < coordinates.length; i++){
			coordinates[i] = CoordinateManager.getInstance().coordinateTransformation(coordinates[i], GENERATOR_COORDINATE_SYSTEM);
		}
		this.coordinates = coordinates;
		this.velocityConstraints = velocityConstraints;

		generatePoints();
		calculatePositions();
		calculateCurvature();
		calculateVelocities();
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
	 * Calculates the base velocity of each point on the path
	 */
	public void calculateVelocities(){



		for (int i = 0; i < trajectoryPoints.length; i++) {
			if(i == 0){
				trajectoryPoints[i].setVelocity(velocityConstraints.getStartVelocity());

			}
			else if(i == trajectoryPoints.length-1){
				trajectoryPoints[i].setVelocity(velocityConstraints.getEndVelocity());
			}
			else {
				trajectoryPoints[i].setVelocity(velocityConstraints.getMaxVelocity());
			}
		}





		for (int i = 1; i < trajectoryPoints.length; i++) {
			double distance = TrajectoryPoint.distance(trajectoryPoints[i - 1], trajectoryPoints[i]);
			double velocity = limitVelocity(trajectoryPoints[i-1].getVelocity(),trajectoryPoints[i].getVelocity(),velocityConstraints.getMaxAcceleration(),distance);
			trajectoryPoints[i].setVelocity(velocity);
		}

		trajectoryPoints[trajectoryPoints.length-1].setVelocity(velocityConstraints.getEndVelocity());

		for (int i = trajectoryPoints.length-2; i > 0; i--) {

			double distance = TrajectoryPoint.distance(trajectoryPoints[i + 1], trajectoryPoints[i]);
			double velocity = Math.min(limitVelocity(trajectoryPoints[i+1].getVelocity(),trajectoryPoints[i].getVelocity(),velocityConstraints.getMaxDeceleration(),distance),trajectoryPoints[i].getVelocity());
			trajectoryPoints[i].setVelocity(velocity);
		}

	}

	private double limitVelocity(double initialVelocity, double desiredVelocity, double acceleration, double distance){

		double time = Math.sqrt((2*distance)/acceleration);

		double sign = Math.signum(initialVelocity-desiredVelocity);

		double velocity = Math.min((initialVelocity + (acceleration*time)), desiredVelocity );

		return velocity;
	}

	/**
	 * Calculates the curvature at each point on the path
	 */
	public void calculateCurvature(){

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
