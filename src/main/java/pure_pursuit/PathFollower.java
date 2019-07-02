package pure_pursuit;


public class PathFollower {
	Path path;
	double lookaheadDistance =  4;
	double currentLookaheadDistance;
	int previousPointIndex = 0;

	private TrajectoryPoint currentClosestPoint;

	double WHEEL_DISTANCE = 26.5;

	private boolean reversed;

	boolean finished = false;

	public PathFollower(Path path){
		new PathFollower(path,false);
	}
	public PathFollower(Path path, boolean reversed){
		this.path=path;
		this.reversed = reversed;
	}

	public TrajectoryFollowerOutput update(){
		double curvature = calculateCurvature(PathFollowerPosition.getInstance().getRobotX(), PathFollowerPosition.getInstance().getRobotY(), PathFollowerPosition.getInstance().getRobotHeading());
		double targetVelocity = findClosestPoint(PathFollowerPosition.getInstance().getRobotX(), PathFollowerPosition.getInstance().getRobotY()).getVelocity();

		double leftVel = targetVelocity*(2+curvature*WHEEL_DISTANCE)/2;
		double rightVel = targetVelocity*(2-curvature*WHEEL_DISTANCE)/2;

		if(reversed){
			leftVel = -leftVel;
			rightVel = -rightVel;
		}

		return new TrajectoryFollowerOutput(leftVel,rightVel);
	}

	private double calculateCurvature(double robotX, double robotY, double robotHeading){
		double a = -Math.tan(Math.toRadians(robotHeading));
		double b = 1;
		double c = Math.tan(Math.toRadians(robotHeading))*robotX-robotY;
		TrajectoryPoint lookaheadPoint = findLookaheadPoint(robotX, robotY);
		double x = Math.abs(a*lookaheadPoint.getX()+b*lookaheadPoint.getY()+c)/Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
		double side = Math.signum(Math.sin(Math.toRadians(robotHeading))*(lookaheadPoint.getX()-robotX)-Math.cos(Math.toRadians(robotHeading))*(lookaheadPoint.getY()-robotY));
		double curvature = 2*x/Math.pow(currentLookaheadDistance,2);
		return curvature*side;
	}



	private TrajectoryPoint findClosestPoint(double x, double y){
		double currentClosest = 1000;
		int index = 0;

		for(int i = 0; i < path.length(); i++){
			if(Math.abs(new TrajectoryPoint(x,y).distance(path.get(i))) < currentClosest){

				currentClosest = Math.abs(new TrajectoryPoint(x,y).distance(path.get(i)));
				index = i;

			}
		}
		previousPointIndex = index;
		currentClosestPoint = path.get(index);
		return path.get(index);
	}
	private TrajectoryPoint findLookaheadPoint(double x, double y){
		double currentClosest = 1000;
		int index = 0;
		TrajectoryPoint closestPoint = findClosestPoint(x,y);
		if(path.get(path.length()-1).distance(closestPoint) <= lookaheadDistance + 1){
			currentLookaheadDistance = lookaheadDistance;
			double a = Math.tan((path.get(path.length()-1).getY()-path.get(path.length()-2).getY())/(path.get(path.length()-1).getX()-path.get(path.length()-2).getX()));
			double x1 = Math.cos(a)*lookaheadDistance;
			double y1 = Math.sin(a)*lookaheadDistance;
			System.out.println("lookahead within dist " + currentLookaheadDistance);
			return new TrajectoryPoint(x1, y1);
		}
		else {
			for (int i = previousPointIndex; i < path.length(); i++) {

				if (Math.abs( path.get(i).distance(new TrajectoryPoint(x, y)) - lookaheadDistance) < currentClosest && path.get(i).distance( new TrajectoryPoint(x, y)) - lookaheadDistance > 0) {
					currentClosest = Math.abs(path.get(i).distance(new TrajectoryPoint(x, y)) - lookaheadDistance);
					currentLookaheadDistance = Math.abs(path.get(i).distance(new TrajectoryPoint(x, y)));
					index = i;
				}
			}
			return path.get(index);
		}

	}

	public TrajectoryPoint getCurrentClosestPoint(){
		return currentClosestPoint;
	}

	public boolean isFinished(){
		return  getCurrentClosestPoint().getX() == path.get(path.length()-1).getX() && getCurrentClosestPoint().getY() == path.get(path.length()-1).getY();
	}

}
