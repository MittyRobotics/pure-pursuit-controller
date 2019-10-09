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
			TurnSign.NEGATIVE,
			VectorDirection.POSITIVE_Y,
			VectorDirection.NEGATIVE_X);

	public Coordinate coordinateTransformation(Coordinate coordinate, CoordinateSystem inputCoordinateSystem) {
		double newX=0;
		double newY=0;
		newX= coordinate.getX();
		newY = coordinate.getY();
		if(inputCoordinateSystem.getForwardVector() != WORLD_COORDINATE_SYSTEM.getForwardVector()){
			if((inputCoordinateSystem.getForwardVector() == VectorDirection.POSITIVE_Y && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.NEGATIVE_Y) ||
					(inputCoordinateSystem.getForwardVector() == VectorDirection.NEGATIVE_Y && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.POSITIVE_Y)){
				newY = -coordinate.getY();
			}
			if((inputCoordinateSystem.getForwardVector() == VectorDirection.POSITIVE_X && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.NEGATIVE_X)||
					(inputCoordinateSystem.getForwardVector() == VectorDirection.NEGATIVE_X && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.POSITIVE_X)){
				newX = -coordinate.getX();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.POSITIVE_Y && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.POSITIVE_X){
				newX = coordinate.getY();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.POSITIVE_Y && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.NEGATIVE_X){
				newX = -coordinate.getY();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.NEGATIVE_Y && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.POSITIVE_X){
				newX = -coordinate.getY();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.NEGATIVE_Y && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.NEGATIVE_X){
				newX = coordinate.getY();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.POSITIVE_X && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.POSITIVE_Y){
				newY = coordinate.getX();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.POSITIVE_X && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.NEGATIVE_Y){
				newY = -coordinate.getX();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.NEGATIVE_X && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.POSITIVE_Y){
				newY = -coordinate.getX();
			}
			if(inputCoordinateSystem.getForwardVector() == VectorDirection.NEGATIVE_X && WORLD_COORDINATE_SYSTEM.getForwardVector() == VectorDirection.NEGATIVE_Y){
				newY = coordinate.getY();
			}
		}
		if(inputCoordinateSystem.getLeftVector() != WORLD_COORDINATE_SYSTEM.getLeftVector()){
			if((inputCoordinateSystem.getLeftVector() == VectorDirection.POSITIVE_Y && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.NEGATIVE_Y) ||
					(inputCoordinateSystem.getLeftVector() == VectorDirection.NEGATIVE_Y && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.POSITIVE_Y)){
				newY = -coordinate.getY();
			}
			if((inputCoordinateSystem.getLeftVector() == VectorDirection.POSITIVE_X && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.NEGATIVE_X)||
					(inputCoordinateSystem.getLeftVector() == VectorDirection.NEGATIVE_X && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.POSITIVE_X)){
				newX = -coordinate.getX();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.POSITIVE_Y && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.POSITIVE_X){
				newX = coordinate.getY();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.POSITIVE_Y && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.NEGATIVE_X){
				newX = -coordinate.getY();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.NEGATIVE_Y && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.POSITIVE_X){
				newX = -coordinate.getY();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.NEGATIVE_Y && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.NEGATIVE_X){
				newX = coordinate.getY();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.POSITIVE_X && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.POSITIVE_Y){
				newY = coordinate.getX();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.POSITIVE_X && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.NEGATIVE_Y){
				newY = -coordinate.getX();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.NEGATIVE_X && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.POSITIVE_Y){
				newY = -coordinate.getX();
			}
			if(inputCoordinateSystem.getLeftVector() == VectorDirection.NEGATIVE_X && WORLD_COORDINATE_SYSTEM.getLeftVector() == VectorDirection.NEGATIVE_Y){
				newY = coordinate.getY();
			}
		}
		double newAngle;
		double mappedAngle = mapAngle(coordinate.getAngle());
		if(inputCoordinateSystem.getLeftTurnSign() != WORLD_COORDINATE_SYSTEM.getLeftTurnSign()){
			if(inputCoordinateSystem.getLeftTurnSign() == TurnSign.NEGATIVE){
				mappedAngle = -mappedAngle;
			}
			else{
				mappedAngle = -mappedAngle;
			}
		}
		newAngle = mapAngle(mappedAngle - (inputCoordinateSystem.getForwardAngle() - WORLD_COORDINATE_SYSTEM.getForwardAngle()));

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
