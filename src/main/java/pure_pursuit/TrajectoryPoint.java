package pure_pursuit;

import java.awt.geom.Point2D;

/**
 * The master object for any point on the path.
 *
 * The point contains an x and y coordinate, a position along the path, a velocity value at this point, and a curvature
 * of the point.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class TrajectoryPoint {

	/**The X position of the point*/
	double x;
	/**The Y position of the point*/
	double y;
	/**The position of the point along the path.*/
	double position;
	/**The velocity of the point.*/
	double velocity;
	/**The curvature of the point.*/
	double curvature;

	/**
	 * Constructor
	 *
	 * @param x the x position of the point.
	 * @param y the y position of the point.
	 */
	public TrajectoryPoint(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Sets the position of the point along the path.
	 * @param position the position along the path of the point.
	 */
	public void setPosition(double position) {
		this.position = position;
	}

	/**
	 * Sets the velocity of the point.
	 * @param velocity the velocity of the point.
	 */
	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	/**
	 * Sets the curvature of the point.
	 * @param curvature the velocity of the point.
	 */
	public void setCurvature(double curvature) {
		this.curvature = curvature;
	}

	/**
	 * returns the x value of the point.
	 * @return the x value of the point.
	 */
	public double getX() {
		return x;
	}

	/**
	 * returns the y value of the point.
	 * @return the y value of the point.
	 */
	public double getY() {
		return y;
	}

	/**
	 * sets the x value of the point.
	 * @param x the x value of the point.
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * sets the y value of the point.
	 * @param y the y value of the point.
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * returns the position along the path of the point.
	 * @return the position along the path of the point.
	 */
	public double getPosition() {
		return position;
	}

	/**
	 * returns the velocity value of the point.
	 * @return the velocity value of the point.
	 */
	public double getVelocity() {
		return velocity;
	}

	/**
	 * returns the curvature value of the point.
	 * @return the curvature value of the point.
	 */
	public double getCurvature() {
		return curvature;
	}


	/**
	 * Returns the distance from this point to another point.
	 *
	 * @param p the point to get the distance to.
	 * @return the distance between this point and p.
	 */
	public double distance(TrajectoryPoint p) {
		return Point2D.distance(this.x, this.y, p.getX(), p.getY());
	}

	/**
	 * Returns the distance between two points.
	 *
	 * @param p1 the first point.
	 * @param p2 the second point.
	 * @return the distance between p1 and p2.
	 */
	public static double distance(TrajectoryPoint p1, TrajectoryPoint p2) {
		return Point2D.distance(p1.getX(), p1.getY(), p2.getX(), p2.getY());
	}

	/**
	 * Gets the midpoint between two points.
	 *
	 * @param p1 the first point.
	 * @param p2 the second point.
	 * @return the midpoint between p1 and p2.
	 */
	public TrajectoryPoint midPoint(TrajectoryPoint p1, TrajectoryPoint p2) {
		TrajectoryPoint point = new TrajectoryPoint((p1.getX() + p2.getX()) / 2, (p1.getY() + p2.getY()) / 2);
		point.setCurvature((p1.getCurvature() + p2.getCurvature()) / 2);
		point.setPosition((p1.getPosition() + p2.getPosition()) / 2);
		point.setVelocity((p1.getVelocity() + p2.getVelocity()) / 2);
		return point;
	}
}
