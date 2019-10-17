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

		}
	}

	/**
	 * Calculates the base velocity of each point on the path
	 */
	public void calculateVelocities(){
		for(int i = 0; i < trajectoryPoints.length; i++){
			trajectoryPoints[i].setVelocity(1);
		}
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
