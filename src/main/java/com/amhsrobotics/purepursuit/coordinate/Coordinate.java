package com.amhsrobotics.purepursuit.coordinate;

import java.awt.geom.Point2D;

public class Coordinate {
	private double x;
	private double y;
	private double angle;

	public Coordinate(double x, double y, double angle){
		this.x = x;
		this.y = y;
		this.angle = angle;
	}

	public double distance(Coordinate coordinate){
		return Point2D.distance(getX(),getY(),coordinate.getX(),coordinate .getY());
	}

	public double distance(Coordinate coordinate1, Coordinate coordinate2){
		return Point2D.distance(coordinate1.getX(),coordinate1.getY(),coordinate2.getX(),coordinate2.getY());
	}

	public double getX() {
		return x;
	}

	public void setX(double x) {
		this.x = x;
	}

	public double getY() {
		return y;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getAngle() {
		return angle;
	}

	public void setAngle(double angle) {
		this.angle = angle;
	}

}
