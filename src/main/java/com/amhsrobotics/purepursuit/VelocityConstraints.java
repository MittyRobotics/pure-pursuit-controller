package com.amhsrobotics.purepursuit;

public class VelocityConstraints {
	private double maxAcceleration;
	private double maxDeceleration;
	private double maxVelocity;
	private double minVelocity;
	private double startVelocity;
	private double endVelocity;
	private double kCurvature;
	
	/**
	 * Velocity constraints for constraining the {@link PurePursuitController}'s output wheel velocity
	 * @param maxAcceleration
	 * @param maxDeceleration
	 * @param maxVelocity
	 */
	public VelocityConstraints(double maxAcceleration, double maxDeceleration, double maxVelocity){
		this(maxAcceleration,maxDeceleration,maxVelocity,0,0,0);
	}
	
	/**
	 * Velocity constraints for constraining the {@link com.amhsrobotics.purepursuit.paths.Path}'s motion profile
	 * @param maxAcceleration
	 * @param maxDeceleration
	 * @param maxVelocity
	 * @param startVelocity
	 * @param endVelocity
	 * @param kCurvature
	 */
	public VelocityConstraints(double maxAcceleration, double maxDeceleration, double maxVelocity, double startVelocity, double endVelocity,  double kCurvature){
		this(maxAcceleration,maxDeceleration,maxVelocity,maxVelocity/2,startVelocity,endVelocity,kCurvature);
	}
	
	/**
	 * Velocity constraints for constraining the {@link com.amhsrobotics.purepursuit.paths.Path}'s motion profile
	 * @param maxAcceleration
	 * @param maxDeceleration
	 * @param maxVelocity
	 * @param minVelocity
	 * @param startVelocity
	 * @param endVelocity
	 * @param kCurvature
	 */
	public VelocityConstraints(double maxAcceleration, double maxDeceleration, double maxVelocity, double minVelocity, double startVelocity, double endVelocity,  double kCurvature){
		this.maxAcceleration = maxAcceleration;
		this.maxDeceleration = maxDeceleration;
		this.maxVelocity = maxVelocity;
		this.minVelocity = minVelocity;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
		this.kCurvature = kCurvature;
	}

	public double getMaxAcceleration() {
		return maxAcceleration;
	}

	public void setMaxAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
	}

	public double getMaxDeceleration() {
		return maxDeceleration;
	}

	public void setMaxDeceleration(double maxDeceleration) {
		this.maxDeceleration = maxDeceleration;
	}

	public double getMaxVelocity() {
		return maxVelocity;
	}

	public void setMaxVelocity(double maxVelocity) {
		this.maxVelocity = maxVelocity;
	}
	public double getMinVelocity() {
		return minVelocity;
	}

	public void setMinVelocity(double minVelocity) {
		this.minVelocity = minVelocity;
	}
	public double getStartVelocity() {
		return startVelocity;
	}

	public void setStartVelocity(double startVelocity) {
		this.startVelocity = startVelocity;
	}

	public double getEndVelocity() {
		return endVelocity;
	}

	public void setEndVelocity(double endVelocity) {
		this.endVelocity = endVelocity;
	}

	public double getkCurvature() {
		return kCurvature;
	}

	public void setkCurvature(double kCurvature) {
		this.kCurvature = kCurvature;
	}


}
