package pure_pursuit;

import java.awt.Point;
import java.awt.geom.Point2D;

public class Waypoint {
	private  Point2D waypoint;
	private  Point2D handle;
	private  Point2D oppositeHandle;
	public Waypoint(Point2D waypoint, Point2D handle){
		this.waypoint = waypoint;
		this.handle = handle;
		this.oppositeHandle = new Point2D.Double(waypoint.getX()+(waypoint.getX()-handle.getX()), waypoint.getY()+(waypoint.getY()-handle.getY()));
	}
	public Point2D getWaypoint(){
		return waypoint;
	}
	public Point2D getHandle(){
		return handle;
	}
	public Point2D getOppositeHandle(){
		return oppositeHandle;
	}
}

