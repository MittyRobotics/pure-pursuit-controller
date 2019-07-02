package pure_pursuit;

public class PathFollowerPosition {
	private static PathFollowerPosition ourInstance = new PathFollowerPosition();

	private double robotX = 0;
	private double robotY = 0;
	private double robotHeading = 0;

	private double resetX = 0;
	private double resetY = 0;
	private double resetHeading = 0;


	public static PathFollowerPosition getInstance() {
		return ourInstance;
	}

	private PathFollowerPosition() {
	}

	public void resetPos(double currentX, double currentY, double currentHeading){
		this.resetX = currentX;
		this.resetY = currentY;
		this.resetHeading = currentHeading;
	}

	public void updatePos(double x, double y, double heading){
		this.robotX = x-resetX;
		this.robotY = y-resetY;
		this.robotHeading = heading-resetHeading;
		if(this.robotHeading < 0){
			this.robotHeading = robotHeading+360;
		}
	}

	public double getRobotX(){
		return robotX;
	}

	public double getRobotY(){
		return robotY ;
	}

	public double getRobotHeading(){
		return robotHeading;
	}
}
