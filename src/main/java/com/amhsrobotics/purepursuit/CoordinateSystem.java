package com.amhsrobotics.purepursuit;

public class CoordinateSystem {
	private double forwardAngle;
	private double leftAngle;
	private double rightAngle;
	private double backwardAngle;
	private VectorDirection forwardVector;
	private VectorDirection leftVector;

	public CoordinateSystem(double forwardAngle, double leftAngle, double rightAngle, double backwardAngle, VectorDirection forwardVector, VectorDirection leftVector){
		this.forwardAngle = forwardAngle;
		this.leftAngle = leftAngle;
		this.rightAngle = rightAngle;
		this.backwardAngle = backwardAngle;
		this.forwardVector = forwardVector;
		this.leftVector = leftVector;
	}

	public double getForwardAngle() {
		return forwardAngle;
	}

	public void setForwardAngle(double forwardAngle) {
		this.forwardAngle = forwardAngle;
	}

	public double getLeftAngle() {
		return leftAngle;
	}

	public void setLeftAngle(double leftAngle) {
		this.leftAngle = leftAngle;
	}

	public double getRightAngle() {
		return rightAngle;
	}

	public void setRightAngle(double rightAngle) {
		this.rightAngle = rightAngle;
	}

	public double getBackwardAngle() {
		return backwardAngle;
	}

	public void setBackwardAngle(double backwardAngle) {
		this.backwardAngle = backwardAngle;
	}

	public VectorDirection getForwardVector() {
		return forwardVector;
	}

	public void setForwardVector(VectorDirection forwardVector) {
		this.forwardVector = forwardVector;
	}

	public VectorDirection getLeftVector() {
		return leftVector;
	}

	public void setLeftVector(VectorDirection backwardVector) {
		this.leftVector = backwardVector;
	}
}
