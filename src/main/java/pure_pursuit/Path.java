package pure_pursuit;

import pure_pursuit.paths.BezierCurvePath;

public class Path {

	TradjectoryPoint[] points;

	BezierCurvePath bezierPath;

	double maxAcceleration, maxVelocity;

	public Path(double maxAcceleration, double maxVelocity, BezierCurvePath bezierPath){
		this.maxAcceleration = maxAcceleration;
		this.maxVelocity = maxVelocity;
		this.bezierPath = bezierPath;
	}

	public void generatePath(){
		points = bezierPath.generate();
	}

	public void calculateDistances(){
		for(int i = 0; i < points.length; i++){
			if(i==0){
				points[i].setPosition(0);
			}
			else{
				points[i].setPosition(points[i-1].getPosition() + TradjectoryPoint.distance(points[i-1],points[i]));
			}
		}
	}

	public double getMaxAcceleration(){
		return maxAcceleration;
	}
	public double getMaxVelocity(){
		return maxVelocity;
	}
	public double length(){
		System.out.println("length " + points.length);
		return points.length;

	}

	public TradjectoryPoint getPoint(int index){
		System.out.println("test " + points[index]);
		return points[index];
	}
}
