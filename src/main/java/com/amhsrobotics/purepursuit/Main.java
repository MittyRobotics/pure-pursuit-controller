package com.amhsrobotics.purepursuit;

public class Main {
	public static void main(String[] args) {
		System.out.println(CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,45), new CoordinateSystem(90,0,180,-90,VectorDirection.POSITIVE_Y, VectorDirection.NEGATIVE_X)).getAngle());
	}
}
