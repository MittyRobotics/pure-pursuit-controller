package pure_pursuit;

public class PathFollower {
	Path path;
	double lookaheadDistance =  1;
	int previousPointIndex = 0;


	double WHEEL_DISTANCE = 3;

	public PathFollower(Path path){
		this.path=path;
	}

	public double[] update(){
		double curvature = calculateCurvature(RobotPose.getInstance().getRobotX(), RobotPose.getInstance().getRobotY(), RobotPose.getInstance().getRobotHeading());
		double targetVelocity = findClosestPoint(RobotPose.getInstance().getRobotX(), RobotPose.getInstance().getRobotY()).getVelocity();
		double leftVel = targetVelocity*(2+curvature*WHEEL_DISTANCE)/2;
		double rightVel = targetVelocity*(2-curvature*WHEEL_DISTANCE)/2;
		return new double[] {leftVel,rightVel};
	}

	public double calculateCurvature(double robotX, double robotY, double robotHeading){
		double a = -Math.tan(Math.toRadians(robotHeading));
		double b = 1;
		double c = Math.tan(Math.toRadians(robotHeading))*robotX-robotY;
		TradjectoryPoint lookaheadPoint = findLookaheadPoint(robotX, robotY);
		double x = Math.abs(a*lookaheadPoint.getX()+b*lookaheadPoint.getY()+c)/Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
		double side = Math.signum(Math.sin(Math.toRadians(robotHeading))*(lookaheadPoint.getX()-robotX)-Math.cos(Math.toRadians(robotHeading))*(lookaheadPoint.getY()-robotY));
		double curvature = 2*x/Math.pow(lookaheadDistance,2);
		return curvature*side;
	}



	public TradjectoryPoint findClosestPoint(double x, double y){
		double currentClosest = 1000;
		int index = 0;
		for(int i = 0; i < path.length(); i++){
			if(Math.abs(new TradjectoryPoint(x,y).distance(path.get(i))) < currentClosest){

				currentClosest = Math.abs(new TradjectoryPoint(x,y).distance(path.get(i)));
				index = i;

			}
		}
		previousPointIndex = index;

		return path.get(index);
	}
	public TradjectoryPoint findLookaheadPoint(double x, double y){
		double currentClosest = 1000;
		int index = 0;
		TradjectoryPoint closestPoint = findClosestPoint(x,y);
		if(path.get(path.length()-1).getPosition()-closestPoint.getPosition() < lookaheadDistance){
			return path.get(path.length()-1);
		}
		else {
			for (int i = previousPointIndex; i < path.length(); i++) {

				if (Math.abs( path.get(i).distance(new TradjectoryPoint(x, y)) - lookaheadDistance) < currentClosest && path.get(i).distance( new TradjectoryPoint(x, y)) - lookaheadDistance > 0) {

					currentClosest = Math.abs(path.get(i).distance(new TradjectoryPoint(x, y)) - lookaheadDistance);

					index = i;
				}
			}
			return path.get(index);
		}

	}
}
