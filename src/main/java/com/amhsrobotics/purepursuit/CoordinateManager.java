package com.amhsrobotics.purepursuit;

public class CoordinateManager {

	private static CoordinateManager instance = new CoordinateManager();

	public static CoordinateManager getInstance() {
		return instance;
	}

	private CoordinateManager() {

	}

	public static CoordinateSystem WORLD_COORDINATE_SYSTEM = new CoordinateSystem(
			0,
			-90,
			90,
			180,
			VectorDirection.POSITIVE_Y,
			VectorDirection.NEGATIVE_X);

	public Coordinate coordinateTransformation(Coordinate coordinate, CoordinateSystem coordinateSystem) {
		double newX=0;
		double newY=0;
		double newAngle;
		double mappedAngle = mapAngle(coordinate.getAngle());
		newAngle = mappedAngle - (coordinateSystem.getForwardAngle() - WORLD_COORDINATE_SYSTEM.getForwardAngle());
		return new Coordinate(newX,newY,newAngle);
	}

	public double mapAngle(double angle) {
		double sign = Math.signum(angle);
		angle = Math.abs(angle % 360);
		if (angle <= 180 && angle >= 0) {
			return angle * sign;
		} else {
			return sign * ((angle % 360) % 180 - 180);
		}


	}
}
