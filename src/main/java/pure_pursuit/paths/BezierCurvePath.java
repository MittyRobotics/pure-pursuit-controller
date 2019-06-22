package pure_pursuit.paths;

import pure_pursuit.BezierPoint;
import pure_pursuit.TradjectoryPoint;

import java.awt.geom.Point2D;

public class BezierCurvePath{
	private int steps;
	private BezierPoint[] waypoints;
	public BezierCurvePath(BezierPoint[] waypoints, int steps){
		this.steps = steps;
		this.waypoints = waypoints;
	}

	public TradjectoryPoint[] generate() {
		TradjectoryPoint[] tradjectoryPoints = new TradjectoryPoint[steps];
		for(int i = 0; i < waypoints.length-1; i++){

			TradjectoryPoint[] segment = generateSegment(waypoints[i], waypoints[i+1], steps/(waypoints.length-1), i==0);
			for(int a = 0; a < segment.length; a++){
				System.out.println(a*(i+1));
				tradjectoryPoints[a*(i+1)] = segment[a];
			}
		}

		return tradjectoryPoints;
	}

	private TradjectoryPoint[] generateSegment(BezierPoint waypoint0, BezierPoint waypoint1, int steps, boolean firstSegment){
		TradjectoryPoint[] tradjectoryPoints = new TradjectoryPoint[steps];
		Point2D p0,p1,p2,p3;
		if(firstSegment){
			p0 = waypoint0.getWaypoint();
			p1 = waypoint0.getHandle();
			p2 = waypoint1.getHandle();
			p3 = waypoint1.getWaypoint();
		}
		else{
			p0 = waypoint0.getWaypoint();
			p1 = waypoint0.getOppositeHandle();
			p2 = waypoint1.getHandle();
			p3 = waypoint1.getWaypoint();
		}

		double t;
		for(int i = 0; i < steps; i++){
			t = (double)i/steps;
			double x = Math.pow(1-t,3)*p0.getX()+3*Math.pow(1-t,2)*t*p1.getX()+3*(1-t)*Math.pow(t,2)*p2.getX()+Math.pow(t,3)*p3.getX();
			double y = Math.pow(1-t,3)*p0.getY()+3*Math.pow(1-t,2)*t*p1.getY()+3*(1-t)*Math.pow(t,2)*p2.getY()+Math.pow(t,3)*p3.getY();
			tradjectoryPoints[i]= new TradjectoryPoint(x,y);
		}
		return tradjectoryPoints;
	}
}
