package pure_pursuit;

import java.awt.geom.Point2D;

public class TradjectoryPoint {
	double x;
	double y;
	double position;
	double velocity;
	double curvature;

	public TradjectoryPoint(double x, double y) {
		this.x = x;
		this.y = y;
	}

	public void setPosition(double position) {
		this.position = position;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	public void setCurvature(double curvature) {
		this.curvature = curvature;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public void setX(double x) {
		this.x = x;
	}

	public void setY(double y) {
		this.y = y;
	}

	public double getPosition() {
		return position;
	}

	public double getVelocity() {
		return velocity;
	}

	public double getCurvature() {
		return curvature;
	}

	public double distance(TradjectoryPoint p) {
		return Point2D.distance(this.x, this.y, p.getX(), p.getY());
	}

	public static double distance(TradjectoryPoint p1, TradjectoryPoint p2) {
		return Point2D.distance(p1.getX(), p1.getY(), p2.getX(), p2.getY());
	}

	public TradjectoryPoint midPoint(TradjectoryPoint p1, TradjectoryPoint p2) {
		TradjectoryPoint point = new TradjectoryPoint((p1.getX() + p2.getX()) / 2, (p1.getY() + p2.getY()) / 2);
		point.setCurvature((p1.getCurvature() + p2.getCurvature()) / 2);
		point.setPosition((p1.getPosition() + p2.getPosition()) / 2);
		point.setVelocity((p1.getVelocity() + p2.getVelocity()) / 2);
		return point;
	}
}
