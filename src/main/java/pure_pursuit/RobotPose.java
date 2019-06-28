package pure_pursuit;

public class RobotPose {
	private static RobotPose ourInstance = new RobotPose();

	private double robotX = 0;
	private double robotY = 0;
	private double robotHeading = 0;

	private double lastLeftEncoderPos = 0;
	private double lastRightEncoderPos = 0;

	private double calibrateGyroVal = 0;

	public static RobotPose getInstance() {
		return ourInstance;
	}

	private RobotPose() {
	}

	public void resetPosition(double currentGyroPos, double currentLeftEncoderPos, double currentRightEncoderPos){
		robotX = 0;
		robotY = 0;
		calibrateGyroVal = currentGyroPos;
		lastLeftEncoderPos = currentLeftEncoderPos;
		lastRightEncoderPos = currentRightEncoderPos;

	}

	public void setPos(double x, double y, double heading){
		this.robotX = x;
		this.robotY = y;
		this.robotHeading = heading;

	}
	public void update(double leftEncoderPos, double rightEncoderPos, double gyroPos, double ticksPerInch){

		//Update robot values based on encoder and gyro output
		robotHeading =gyroPos - calibrateGyroVal;
		if(robotHeading < 0){
			robotHeading = robotHeading+360;
		}
		robotHeading = 0-robotHeading;
		double deltaLeftPos = leftEncoderPos - lastLeftEncoderPos;
		double deltaRightPos = rightEncoderPos - lastRightEncoderPos;
		double deltaPosition = (deltaLeftPos + deltaRightPos)/2/ticksPerInch;
		robotX += deltaPosition * Math.cos(Math.toRadians(robotHeading));
		robotY += deltaPosition * Math.sin(Math.toRadians(robotHeading));
		lastLeftEncoderPos = leftEncoderPos;
		lastRightEncoderPos = rightEncoderPos;
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
