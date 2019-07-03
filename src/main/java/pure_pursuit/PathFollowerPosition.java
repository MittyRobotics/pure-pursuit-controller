package pure_pursuit;

/**
 * An object that keeps track of the robot's current position.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class PathFollowerPosition {
	private static PathFollowerPosition ourInstance = new PathFollowerPosition();

	/**The robot's current X position relative to the starting point of the path.*/
	private double robotX = 0;
	/**The robot's current Y position relative to the starting point of the path.*/
	private double robotY = 0;
	/**The robot's current heading angle (degrees) relative to the starting point of the path.*/
	private double robotHeading = 0;

	/**The robot's X position at the start of the path*/
	private double resetX = 0;
	/**The robot's Y position at the start of the path*/
	private double resetY = 0;
	/**The robot's heading angle (degrees) at the start of the path*/
	private double resetHeading = 0;


	/**
	 * Returns the singleton instance.
	 * @return the instance of the {@link PathFollowerPosition}.
	 */
	public static PathFollowerPosition getInstance() {
		return ourInstance;
	}

	/**
	 * Constructor
	 */
	private PathFollowerPosition() {
	}

	/**
	 * Resets the position of the robot.
	 * @param currentX the current robot X position.
	 * @param currentY the current robot Y position.
	 * @param currentHeading the current robot heading angle (degrees).
	 */
	public void resetPos(double currentX, double currentY, double currentHeading){
		this.resetX = currentX;
		this.resetY = currentY;
		this.resetHeading = currentHeading;
	}

	/**
	 * Updates the position of the robot relative to the start of the path.
	 *
	 * This calculates the robot's current position relative to the start of the path, meaning the robot's initial
	 * position once the path following starts is (0,0,0).
	 * @param x the robot's current X value.
	 * @param y the robot's current Y value.
	 * @param heading the robot's current heading angle (degrees).
	 */
	public void updatePos(double x, double y, double heading){
		this.robotX = x-resetX;
		this.robotY = y-resetY;
		this.robotHeading = heading-resetHeading;
		if(this.robotHeading < 0){
			this.robotHeading = robotHeading+360;
		}
	}

	/**
	 * Returns the robot's current X value relative to the start of the path.
	 * @return the robot's current X value.
	 */
	public double getRobotX(){
		return robotX;
	}

	/**
	 * Returns the robot's current Y value relative to the start of the path.
	 * @return the robot's current Y value.
	 */
	public double getRobotY(){
		return robotY ;
	}

	/**
	 * Returns the robot's current heading angle relative to the start of the path.
	 * @return the robot's current heading angle (degrees).
	 */
	public double getRobotHeading(){
		return robotHeading;
	}
}
