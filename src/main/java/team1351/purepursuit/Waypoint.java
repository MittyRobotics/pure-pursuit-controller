package team1351.purepursuit;

import java.awt.geom.Point2D;

/**
 * The master waypoint object.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class Waypoint {
	/**The waypoint {@link Point2D}. This is the main position of the waypoint.*/
	private  Point2D waypoint;
	/**
	 * The handle {@link Point2D}. This is the handle of the waypoint used in {@link team1351.purepursuit.paths.BezierCurvePath}.
	 * For {@link team1351.purepursuit.paths.LinearPath} the handle is included as another waypoint.
	 */
	private  Point2D handle;
	/**The opposite handle {@link Point2D}. This is the point directly opposite of the handle point.*/
	private  Point2D oppositeHandle;

	/**
	 * Constructor
	 *
	 * @param waypoint the waypoint point.
	 * @param handle the handle point.
	 */
	public Waypoint(Point2D waypoint, Point2D handle){
		this.waypoint = waypoint;
		this.handle = handle;
		this.oppositeHandle = new Point2D.Double(waypoint.getX()+(waypoint.getX()-handle.getX()), waypoint.getY()+(waypoint.getY()-handle.getY()));
	}

	/**
	 * Returns the waypoint point.
	 *
	 * @return the waypoint point
	 */
	public Point2D getWaypoint(){
		return waypoint;
	}

	/**
	 * Returns the handle point.
	 *
	 * @return the handle point
	 */
	public Point2D getHandle(){
		return handle;
	}

	/**
	 * Returns the opposite handle point.
	 *
	 * @return the opposite handle point
	 */
	public Point2D getOppositeHandle(){
		return oppositeHandle;
	}
}

