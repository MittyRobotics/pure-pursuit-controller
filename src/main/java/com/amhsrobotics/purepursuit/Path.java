package com.amhsrobotics.purepursuit;


import com.amhsrobotics.purepursuit.paths.BezierCurvePath;
import com.amhsrobotics.purepursuit.paths.CubicHermiteSplinePath;
import com.amhsrobotics.purepursuit.paths.LinearPath;

/**
 * Master Path Object.
 * <p>
 * Contains functions for generating the distance, curvature, and velocity for each point. Holds the master array of
 * {@link TrajectoryPoint} for the path with calculated x, y, distance, curvature, and velocity values. Some calculations
 * are based off of team 1712's paper: https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
 *
 * @author Owen Leather
 * @version 1.0
 */
public class Path {

	/** Array of {@link TrajectoryPoint}s that make up the path.*/
	private TrajectoryPoint[] points;
    /** Generated bezier curve path object.*/
	private BezierCurvePath bezierPath;
	private CubicHermiteSplinePath hermitePath;
	/** Generated linear path object.*/
	private LinearPath linearPath;

	/** Maximum robot acceleration value. This is used to limit the velocity of each point based on the acceleration.*/
	private double maxAcceleration;
	private double maxDeceleration;
	/** Maximum robot velocity value. This is used to make sure the velocity of each point does not go over the maximum
	 * velocity of the robot.
	 */
	private double maxVelocity;
	/**The robot's starting velocity on the path. This is used when the robot starts the path while moving at a certain forward velocity*/
	private double startVelocity;
	/**The robot's ending velocity on the path. This is used when the robot should end the path moving at a certain forward velocity*/
	private double endVelocity;

	/**
	 * Value that determines how much the velocity slows down around turns. Decreasing this value makes the velocity slow down
	 * more around turns. This value is usually best around 0.8-2, 0.8 tends to slow down around almost any curvature in the
	 * path, and 2 tends to slow down around only very sharp curvature.
	 */
	private double kCurvature;

	/**
	 * Constructor
	 * <p>
	 * Sets up a path with a {@link BezierCurvePath} as the path generation method.
	 *
	 * @param maxAcceleration Maximum robot acceleration.
	 * @param maxVelocity     Maximum robot velocity.
	 * @param bezierPath      bezier curve path.
	 */
	public Path(double maxAcceleration, double maxVelocity, BezierCurvePath bezierPath) {
		this(maxAcceleration,maxVelocity,0,0,bezierPath);
	}

	/**
	 * Constructor
	 * <p>
	 * Sets up a path with a {@link LinearPath} as the path generation method.
	 *
	 * @param maxAcceleration Maximum robot acceleration.
	 * @param maxVelocity     Maximum robot velocity.
	 * @param linearPath      linear path.
	 */
	public Path(double maxAcceleration, double maxVelocity, LinearPath linearPath) {
		this(maxAcceleration,maxVelocity,0,0,linearPath);
	}

	/**
	 * Constructor
	 * <p>
	 * Sets up a path with a {@link LinearPath} as the path generation method.
	 *
	 * @param maxAcceleration Maximum robot acceleration.
	 * @param maxVelocity     Maximum robot velocity.
	 * @param startVelocity   the robot starting forward velocity.
	 * @param endVelocity     the robot ending forward velocity.
	 * @param bezierPath      bezier curve path.
	 */
	public Path(double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity, BezierCurvePath bezierPath) {
		this.maxAcceleration = maxAcceleration;
		this.maxVelocity = maxVelocity;
		this.bezierPath = bezierPath;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
	}

	/**
	 * Constructor
	 * <p>
	 * Sets up a path with a {@link LinearPath} as the path generation method.
	 *
	 * @param maxAcceleration Maximum robot acceleration.
	 * @param maxVelocity     Maximum robot velocity.
	 * @param startVelocity   the robot starting forward velocity.
	 * @param endVelocity     the robot ending forward velocity.
	 * @param hermitePath      hermite spline path.
	 */
	public Path(double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity, CubicHermiteSplinePath hermitePath) {
		this.maxAcceleration = maxAcceleration;
		this.maxVelocity = maxVelocity;
		this.hermitePath = hermitePath;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
	}

	/**
	 * Constructor
	 * <p>
	 * Sets up a path with a {@link LinearPath} as the path generation method.
	 *
	 * @param maxAcceleration Maximum robot acceleration.
	 * @param maxVelocity     Maximum robot velocity.
	 * @param startVelocity   the robot starting forward velocity.
	 * @param endVelocity     the robot ending forward velocity.
	 * @param linearPath      linear path.
	 */
	public Path(double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity, LinearPath linearPath) {
		this.maxAcceleration = maxAcceleration;
		this.maxVelocity = maxVelocity;
		this.linearPath = linearPath;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
	}

	public Path(double maxAcceleration, double maxDeceleration, double maxVelocity, double startVelocity, double endVelocity, CubicHermiteSplinePath hermitePath) {
		this.maxAcceleration = maxAcceleration;
		this.maxDeceleration = maxDeceleration;
		this.maxVelocity = maxVelocity;
		this.hermitePath = hermitePath;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
	}

	/**
	 * Function for calling an individual path generator.
	 * <p>
	 * Calls the generation function for the path type that is used for this path.
	 */
	public void generatePath() {
		if (bezierPath != null) {
			points = bezierPath.generate();
		} else if(hermitePath != null){
			points = hermitePath.generate();
		} else if (linearPath != null) {
			points = linearPath.generate();
		}

	}

	/**
	 * Calculates the distances of each point along the path.
	 */
	public void calculateDistances() {
		for (int i = 0; i < points.length; i++) {
			if (i == 0) {
				points[i].setPosition(0);
			} else {
				points[i].setPosition(points[i - 1].getPosition() + TrajectoryPoint.distance(points[i - 1], points[i]));
			}
		}
	}

	/**
	 * Calculates the curvature for each point along the path.
	 * <p>
	 * The curvature is used to slow down the robot's velocity around corners. The curvature calculation is done by
	 * taking the previous, current, and next point of the {@link Path} and fitting a circle through those points. The
	 * curvature is then 1/radius of the fit circle.
	 */
	public void calculateCurvature() {
		for (int i = 0; i < points.length; i++) {
			if (i == 0) {
				points[i].setCurvature(0);
			} else if (i == points.length - 1) {
				points[i].setCurvature(0);
			} else {
				double x1 = points[i - 1].getX();
				double y1 = points[i - 1].getY();
				double x2 = points[i].getX();
				double y2 = points[i].getY();
				double x3 = points[i + 1].getX();
				double y3 = points[i + 1].getY();

				if (x1 == x2) {
					//Avoid divide by 0 errors
					x2 += 0.00001;
				}

				double k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2)) / (x1 - x2);
				double k2 = (y1 - y2) / (x1 - x2);
				double b = 0.5 * (Math.pow(x2, 2) - 2 * x2 * k1 + Math.pow(y2, 2) - Math.pow(x3, 2) + 2 * x3 * k1 - Math.pow(y3, 2)) / (x3 * k2 - y3 + y2 - x2 * k2);
				double a = k1 - k2 * b;
				double r = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(y1 - b, 2));
				double curvature = 1 / r;

				if (Double.isNaN(r)) {
					curvature = 0;
				}
				points[i].setCurvature(curvature);

			}
		}
	}

	/**
	 * Calculates the velocity for each point along the path.
	 * <p>
	 * This makes sure the robot never accelerates faster than the maxAcceleration and never goes faster than
	 * maxVelocity. It also takes the curvature of each point into account to slow down the velocity around turning.
	 * <p>
	 * The function starts by looping through the points backwards and calculating the velocity based on acceleration
	 * and curvature. Then it loops through the points forwards doing the same thing. The reason it loops through the
	 * points twice is because the algorithm for calculating the velocity based on acceleration only works on
	 * acceleration periods.
	 */

	public void calculateVelocities() {



		for (int i = 0; i < points.length; i++) {
			if(i == 0){
				points[i].setVelocity(startVelocity);
			}
			else if(i == points.length-1){
				points[i].setVelocity(endVelocity);
			}
			else {
				points[i].setVelocity(maxVelocity);
			}
		}



		//			double maxVelocityWithCurvature = Math.min(maxVelocity, kCurvature / points[i].getCurvature());
//			if (i == points.length - 1) {
//				points[i].setVelocity(endVelocity);
//			} else {
//				double distance = TrajectoryPoint.distance(points[i + 1], points[i]);
//				double velocity;
//				if(maxDeceleration != 0){
////					velocity = Math.min(maxVelocityWithCurvature, Math.sqrt(Math.pow(points[i + 1].getVelocity(), 2) + 2 * maxDeceleration * distance));
//					velocity = limitVelocity(points[i+1].getVelocity(),maxVelocity,maxAcceleration,distance);
////					System.out.println(points[i + 1].getVelocity());
//				}
//				else {
////					velocity = Math.min(maxVelocityWithCurvature, Math.sqrt(Math.pow(points[i + 1].getVelocity(), 2) + 2 * maxAcceleration * distance));
//					velocity = limitVelocity(points[i+1].getVelocity(),maxVelocity,maxAcceleration,distance);
////					System.out.println(points[i + 1].getVelocity());
//				}
//				points[i].setVelocity(velocity);
//			}




		for (int i = 1; i < points.length; i++) {
			double maxVelocityWithCurvature = kCurvature / points[i].getCurvature();

			maxVelocityWithCurvature = Math.round(maxVelocityWithCurvature/(maxVelocity/5)) *  (maxVelocity/5);

			double distance = TrajectoryPoint.distance(points[i - 1], points[i]);
			double velocity = limitVelocity(Math.min(points[i-1].getVelocity(),maxVelocityWithCurvature),points[i].getVelocity(),maxAcceleration,distance);
			points[i].setVelocity(velocity);
		}

		points[points.length-1].setVelocity(endVelocity);

		for (int i = points.length-2; i > 0; i--) {
			double maxVelocityWithCurvature = kCurvature / points[i].getCurvature();

			maxVelocityWithCurvature = Math.round(maxVelocityWithCurvature/(maxVelocity/5)) * (maxVelocity/5);

			double distance = TrajectoryPoint.distance(points[i + 1], points[i]);
			double velocity = Math.min(limitVelocity(Math.min(points[i+1].getVelocity(),maxVelocityWithCurvature),points[i].getVelocity(),maxDeceleration,distance),points[i].getVelocity());
			points[i].setVelocity(velocity);
		}



//		for (int i = 0; i < points.length; i++) {
//			if (i == 0) {
//				points[i].setVelocity(maxAcceleration+startVelocity);
//			} else {
//				double distance = TrajectoryPoint.distance(points[i - 1], points[i]);
////				double velocity = Math.min(points[i].getVelocity(), Math.sqrt(Math.pow(points[i - 1].getVelocity(), 2) + 2 * maxAcceleration * distance));
////				double velocity = limitVelocity(points[i-1].getVelocity(),endVelocity,maxAcceleration,distance);
//////				System.out.println(points[i - 1].getVelocity());
////				points[i].setVelocity(velocity);
//			}
//		}
	}

	private double limitVelocity(double initialVelocity, double desiredVelocity, double acceleration, double distance){

		double time = Math.sqrt((2*distance)/acceleration);

		double sign = Math.signum(initialVelocity-desiredVelocity);

		double velocity = Math.min((initialVelocity + (acceleration*time)), desiredVelocity );


		System.out.println(time +  " " + initialVelocity + " " + desiredVelocity + " " + acceleration + " " + distance + " " + velocity);

		return velocity;
	}

	/**
	 * Returns maxAcceleration.
	 * @return maximum acceleration assigned to the path.
	 */
	public double getMaxAcceleration() {
		return maxAcceleration;
	}
	/**
	 * Returns maxVelocity.
	 * @return maximum velocity assigned to the path.
	 */
	public double getMaxVelocity() {
		return maxVelocity;
	}

	/**
	 * Sets the kCurvature value.
	 *
	 * kCurvature is how much the velocity slows down around turns. Decreasing this value makes the velocity slow down
	 * more around turns. This value is usually best around 0.8-2, 0.8 tends to slow down around almost any curvature in the
	 * path, and 2 tends to slow down around only very sharp curvature.
	 *
	 * @param kCurvature the kCurvature value.
	 */
	public void setKCurvature(double kCurvature) {
		this.kCurvature = kCurvature;
	}

	/**
	 * Returns the kCurvature value assigned to this path.
	 * @return kCurvature.
	 */
	public double getKCurvature() {
		return kCurvature;
	}

	/**
	 * Returns the amount of points in the path.
	 * @return the length of the array of points.
	 */
	public int length() {
		return points.length;
	}

	/**
	 * Returns the {@link TrajectoryPoint} at the specified index.
	 * @param index the index of the returned {@link TrajectoryPoint}.
	 * @return the {@link TrajectoryPoint} at the specified index.
	 */
	public TrajectoryPoint get(int index) {
		return points[index];
	}
}
