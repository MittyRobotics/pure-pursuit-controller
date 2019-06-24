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
	
	public void calculateCurvature(){
		for(int i = 0; i < points.length; i++){
			if(i == 0){
				points[i].setCurvature(0);
			}
			else if(i==points.length-1){
				points[i].setCurvature(0);
			}
			else{
				double x1 = points[i-1].getX();
				double y1 = points[i-1].getY();
				double x2 = points[i].getX();
				double y2 = points[i].getY();
				double x3 = points[i+1].getX();
				double y3 = points[i+1].getY();

				if(x1 == x2){
					//Avoid divide by 0 errors
					x2 += 0.00001;
				}

				double k1 = 0.5*(Math.pow(x1,2)+Math.pow(y1,2)-Math.pow(x2,2)-Math.pow(y2,2))/(x1-x2);
				double k2 = (y1-y2)/(x1-x2);
				double b = 0.5 * (Math.pow(x2,2)-2*x2*k1+Math.pow(y2,2)-Math.pow(x3,2)+2*x3*k1-Math.pow(y3,2))/(x3*k2-y3+y2-x2*k2);
				double a = k1-k2*b;
				double r = Math.sqrt(Math.pow(x1-a,2)+Math.pow(y1-b,2));
				double curvature = 1/r;

				if(Double.isNaN(r)) {
					curvature = 0;
				}
				points[i].setCurvature(curvature);

			}
		}
	}

	public void calculateVelocities(){
		double kCurvature = 0.4; //Intensity of slowing down at turns
		for(int i = points.length-1; i > 0; i--){
			double maxVelocityWithCurvature = Math.min(maxVelocity, kCurvature/points[i].getCurvature());
			if(i == points.length-1){
				points[i].setVelocity(0);
			}
			else{
				double distance =TradjectoryPoint.distance(points[i+1], points[i]);
				double velocity = Math.min(maxVelocityWithCurvature, Math.sqrt(Math.pow(points[i+1].getVelocity(),2)+2*maxAcceleration*distance));
				points[i].setVelocity(velocity);
			}
		}
		for(int i =0; i < points.length; i++){
			if(i == 0){
				points[i].setVelocity(0.6);
			}
			else{
				double distance = TradjectoryPoint.distance(points[i-1], points[i]);
				double velocity = Math.min(points[i].getVelocity(), Math.sqrt(Math.pow(points[i-1].getVelocity(),2)+2*maxAcceleration*distance));
				points[i].setVelocity(velocity);
			}
		}
	}

	public double getMaxAcceleration(){
		return maxAcceleration;
	}
	public double getMaxVelocity(){
		return maxVelocity;
	}
	
	public int length(){
		return points.length;
	}

	public TradjectoryPoint get(int index){
		return points[index];
	}
}
