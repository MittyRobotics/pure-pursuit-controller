package com.amhsrobotics.purepursuit;

public class CoordinateSystem {

	private double forwardAngle;

	private TurnSign leftTurnSign;
	private VectorDirection forwardVector;
	private VectorDirection leftVector;

	public CoordinateSystem(double forwardAngle, TurnSign leftTurnSign, VectorDirection forwardVector, VectorDirection leftVector) {
		this.forwardAngle = forwardAngle;
		this.leftTurnSign = leftTurnSign;
		this.forwardVector = forwardVector;
		this.leftVector = leftVector;
	}

	public double getForwardAngle() {
		return forwardAngle;
	}

	public void setForwardAngle(double forwardAngle) {
		this.forwardAngle = forwardAngle;
	}


	public TurnSign getLeftTurnSign() {
		return leftTurnSign;
	}

	public void setLeftTurnSign(TurnSign leftTurnSign) {
		this.leftTurnSign = leftTurnSign;
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
