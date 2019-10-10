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

	private CoordinateSystem GENERATOR_COORDINATE_SYTEM = new CoordinateSystem(90, TurnSign.NEGATIVE, VectorDirection.NEGATIVE_Y,VectorDirection.NEGATIVE_X);

	/**
	 * Path constructor
	 */
	public Path(Coordinate[] coordinates, VelocityConstraints velocityConstraints){

		for(int i = 0; i < coordinates.length; i++){
			coordinates[i] = CoordinateManager.getInstance().coordinateTransformation(coordinates[i],GENERATOR_COORDINATE_SYTEM);
			System.out.println(coordinates[i].getX() + " " + coordinates[i].getY() + " " + coordinates[i].getAngle());
		}
		this.coordinates = coordinates;
		this.velocityConstraints = velocityConstraints;
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

	}

	/**
	 * Calculates the base velocity of each point on the path
	 */
	public void calculateVelocities(){

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
