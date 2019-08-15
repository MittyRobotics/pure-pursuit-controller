package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.paths.BezierCurvePath;
import com.amhsrobotics.purepursuit.paths.CubicHermiteSplinePath;

import java.awt.geom.Point2D;

/**
 * The waypoint object for all path generation methods.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class Waypoint {
	/**The waypoint {@link Point2D}. This is the main position of the waypoint.*/
	private  Point2D waypoint;
	/**The handle {@link Point2D}. This is the handle of the waypoint used in {@link BezierCurvePath}.*/
	private  Point2D handle;
	/**The opposite handle {@link Point2D}. This is the point directly opposite of the handle point.*/
	private  Point2D oppositeHandle;
	/**
	 * The angle of the tangent vector at the waypoint in degrees. This is used in
	 * {@link CubicHermiteSplinePath} to determine the tangent vector.
	 */
	private double angle;

	/**
	 * Constructor
	 *
	 * This contains a waypoint point and a handle point. This is primarily used for the bezier curve interpolation.
	 *
	 * @param waypoint the waypoint point.
	 * @param handle the handle point.
	 */
	public Waypoint(Point2D waypoint, Point2D handle){
		this.waypoint = waypoint;
		this.handle = handle;
		this.oppositeHandle = new Point2D.Double(waypoint.getX()+(waypoint.getX()-handle.getX()), waypoint.getY()+(waypoint.getY()-handle.getY()));
		this.angle = Math.toDegrees(Math.atan2(handle.getY()-waypoint.getY(),handle.getX()-waypoint.getX()));
	}

	/**
	 * Constructor
	 *
	 * This contains a waypoint point and an angle, and is primarily used for the cubic hermite spline interpolation.
	 *
	 * @param waypoint the waypoint point.
	 * @param angle the angle of the tangent vector at this waypoint.
	 */
	public Waypoint(Point2D waypoint, double angle){
		this.waypoint = waypoint;
		this.handle = null;
		this.oppositeHandle = null;
		this.angle = angle;
	}

	/**
	 * Constructor
	 *
	 * This contains only a waypoint point, and is primarily used for the linear interpolation generation method.
	 *
	 * @param waypoint the waypoint point.
	 */
	public Waypoint(Point2D waypoint){
		this.waypoint = waypoint;
		this.handle = null;
		this.oppositeHandle = null;
		this.angle = 0;
	}

	/**
	 * Returns the waypoint point.
	 *
	 * @return the waypoint point.
	 */
	public Point2D getWaypoint(){
		return waypoint;
	}

	/**
	 * Returns the handle point.
	 *
	 * @return the handle point.
	 */
	public Point2D getHandle(){
		return handle;
	}

	/**
	 * Returns the opposite handle point.
	 *
	 * @return the opposite handle point.
	 */
	public Point2D getOppositeHandle(){
		return oppositeHandle;
	}

	/**
	 * Returns the angle of the tangent vector at the point.
	 *
	 * @return the angle of the tangent vector at the point.
	 */
	public double getAngle(){
		return angle;
	}
}

