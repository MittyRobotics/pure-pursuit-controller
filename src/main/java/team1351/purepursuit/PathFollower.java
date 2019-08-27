package team1351.purepursuit;

/**
 * Path Follower Object that allows the robot to follow the path.
 *
 * This contains functions for generating the left and right feed-forward wheel velocities for the wheels based on a given
 * robot position and heading. Some calculations are based off of team 1712's paper:
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
 *
 * @author Owen Leather
 * @version 1.0
 */
public class PathFollower {
	/**Current path to follow*/
	private Path path;
	/**
	 * Lookahead distance of the path follower. The lookahead distance determines the distance that the lookahead point
	 * is away from the robot. The higher the lookahead distance, the larger the turns the robot makes are. More information at:
	 * https://www.mathworks.com/help/robotics/ug/pure-pursuit-controller.html#burwbfa
	 */
	private double lookaheadDistance;
	/**The current lookahead distance that is slightly changed based on the closest pre-generated point.*/
	private double currentLookaheadDistance;
	/**The index of the previous closest point.*/
	private int previousPointIndex = 0;

	/**The current closest {@link TrajectoryPoint} to the robot. This determines the robot's base velocity.*/
	private TrajectoryPoint currentClosestPoint;

	/**The distance between the wheels on the chassis, also known as the track width.*/
	private double WHEEL_DISTANCE;

	/**If the robot should follow the path reversed. If this is true, the robot will be following the path backwards*/
	private boolean reversed;

	private double currentCurvature;

	private TrajectoryPoint currentLLookaheadPoint;

	/**
	 * Constructor
	 *
	 * @param path the path to follow
	 */
	public PathFollower(Path path){
		this(path,false);
	}

	/**
	 * Constructor
	 *
	 * @param path the path to follow
	 * @param reversed whether or not the output should be reversed, resulting in the robot following the path backwards.
	 */
	public PathFollower(Path path, boolean reversed){
		this.path=path;
		this.reversed = reversed;
	}

	/**
	 * Updates the path follower.
	 *
	 * Master update function to return the left and right feed-forward wheel velocity values. These values are returned in the form of a {@link PathFollowerOutput}.
	 *
	 * @return A {@link PathFollowerOutput} object with the left and right feed-forward wheel velocity values.
	 */
	public PathFollowerOutput update(){
		double curvature = calculateCurvature(PathFollowerPosition.getInstance().getRobotX(), PathFollowerPosition.getInstance().getRobotY(), PathFollowerPosition.getInstance().getRobotHeading());
		double targetVelocity = findClosestPoint(PathFollowerPosition.getInstance().getRobotX(), PathFollowerPosition.getInstance().getRobotY()).getVelocity();
		this.currentCurvature = curvature;

		double leftVel = targetVelocity*(2+(curvature*WHEEL_DISTANCE))/2;
		double rightVel = targetVelocity*(2-(curvature*WHEEL_DISTANCE))/2;

		if(reversed){
			leftVel = -leftVel;
			rightVel = -rightVel;
		}

		return new PathFollowerOutput(leftVel,rightVel);
	}

	/**
	 * Calculates the curvature of the arc from the robot to the lookahead point.
	 *
	 * @param robotX robot's current X position.
	 * @param robotY robot's current Y position.
	 * @param robotHeading robot's current heading angle (degrees).
	 * @return the curvature of the arc from the robot's position to the lookahead point.
	 */
	private double calculateCurvature(double robotX, double robotY, double robotHeading){
		double a = -Math.tan(Math.toRadians(robotHeading));
		double b = 1;
		double c = Math.tan(Math.toRadians(robotHeading))*robotX-robotY;
		TrajectoryPoint lookaheadPoint = findLookaheadPoint(robotX, robotY);
		this.currentLLookaheadPoint = lookaheadPoint;
		double x = Math.abs(a*lookaheadPoint.getX()+b*lookaheadPoint.getY()+c)/Math.sqrt(Math.pow(a,2) + Math.pow(b,2));
		double side = Math.signum(Math.sin(Math.toRadians(robotHeading))*(lookaheadPoint.getX()-robotX)-Math.cos(Math.toRadians(robotHeading))*(lookaheadPoint.getY()-robotY));
		double curvature = 2*x/Math.pow(currentLookaheadDistance,2);
		return curvature*side;
	}

	/**
	 * Finds the closest point to the robot.
	 *
	 * This point is used to determine the robot's base forward velocity.
	 *
	 * @param x the robot's current X position.
	 * @param y the robot's current Y position.
	 * @return a {@link TrajectoryPoint} that is the closest point on the path to the robot.
	 */
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

	/**
	 * Finds the lookahead point based on the robot's current position and the lookahead distance.
	 *
	 * The lookahead point is the point that the robot follows on the path. This function finds the closest point on the
	 * path that is ahead of the {@link #findClosestPoint(double, double)} that is lookahead distance away from the robot.
	 *
	 * @param x the robot's current X position.
	 * @param y the robot's current Y position.
	 * @return a {@link TrajectoryPoint} that is the lookahead point on the path.
	 */
	private TrajectoryPoint findLookaheadPoint(double x, double y){
		double currentClosest = 1000;
		int index = 0;
		TrajectoryPoint closestPoint = findClosestPoint(x,y);
		if(path.get(path.length()-1).distance(closestPoint) <= lookaheadDistance + 1){
			currentLookaheadDistance = lookaheadDistance;
			double a = Math.atan2((path.get(path.length()-1).getY()-path.get(path.length()-2).getY()),(path.get(path.length()-1).getX()-path.get(path.length()-2).getX()));
			double xOffset = (path.get(path.length()-1).getX()-path.get(path.length()-2).getX());
			double yOffset = (path.get(path.length()-1).getY()-path.get(path.length()-2).getY());
			double x1 = path.get(path.length()-1).getX() + xOffset*lookaheadDistance;
			double y1 =path.get(path.length()-1).getY() + yOffset*lookaheadDistance;
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

	/**
	 * Returns the current closest point to the robot.
	 *
	 * @return the {@link TrajectoryPoint} that is the current closest point on the path to the robot.
	 */
	public TrajectoryPoint getCurrentClosestPoint(){
		return currentClosestPoint;
	}

	/**
	 * Sets the lookahead distance value.
	 *
	 * @param lookaheadDistance the lookahead distance value.
	 */
	public void setLookaheadDistance(double lookaheadDistance){
		this.lookaheadDistance = lookaheadDistance;
	}

	/**
	 * Returns the lookahead distance value.
	 *
	 * @return the lookahead distance value.
	 */
	public double getLookaheadDistance(){
		return lookaheadDistance;
	}

	/**
	 * Sets the wheel distance value.
	 *
	 * The wheel distance is the distance between the wheels on the chassis, also known as the track width.
	 *
	 * @param WHEEL_DISTANCE the wheel distance value.
	 */
	public void setWheelDistance(double WHEEL_DISTANCE){
		this.WHEEL_DISTANCE = WHEEL_DISTANCE;
	}

	/**
	 * Returns the wheel distance value of the path follower.
	 * @return the wheel distance value.
	 */
	public double getWheelDistance(){
		return WHEEL_DISTANCE;
	}

	public double getCurvature(){
		return currentCurvature;
	}

	public TrajectoryPoint getCurrentLookaheadPoint(){
		return currentLLookaheadPoint;
	}

	/**
	 * Function that checks whether or not the path following is finished.
	 *
	 * This is determined by if the {@link #currentClosestPoint} is the last point of the {@link #path}.
	 *
	 * @return if the path follower is finished following the path.
	 */
	public boolean isFinished(){
		return  getCurrentClosestPoint().getX() == path.get(path.length()-1).getX() && getCurrentClosestPoint().getY() == path.get(path.length()-1).getY();
	}

}
