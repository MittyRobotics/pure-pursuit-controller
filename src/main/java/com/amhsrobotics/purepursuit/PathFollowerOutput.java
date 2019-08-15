package com.amhsrobotics.purepursuit;

/**
 * Path follower output object.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class PathFollowerOutput {
	/**The left wheel feed-forward velocity value.*/
	private double leftVelocity;
	/**The right wheel feed-forward velocity value.*/
	private double rightVelocity;

	/**
	 * Constructor
	 *
	 * @param leftVelocity left wheel feed-forward velocity value.
	 * @param rightVelocity right wheel feed-forward velocity value.
	 */
	public PathFollowerOutput(double leftVelocity, double rightVelocity){
		this.leftVelocity = leftVelocity;
		this.rightVelocity = rightVelocity;
	}

	/**
	 * Returns the left wheel feed-forward velocity value.
	 *
	 * @return the left wheel feed-forward velocity value.
	 */
	public double getLeftVelocity(){
		return leftVelocity;
	}

	/**
	 * Returns the right wheel feed-forward velocity value.
	 *
	 * @return the right wheel feed-forward velocity value.
	 */
	public double getRightVelocity(){
		return rightVelocity;
	}

}
