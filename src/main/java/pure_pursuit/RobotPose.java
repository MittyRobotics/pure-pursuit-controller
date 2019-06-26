package pure_pursuit;

import java.util.TimerTask;

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

	public void resetPosition(){
		robotX = 0;
		robotY = 0;
//		calibrateGyroVal = Gyro.getInstance().getAngle();
//		lastLeftEncoderPos = DriveTrain.getInstance().getLeftEncoder();
//		lastRightEncoderPos = DriveTrain.getInstance().getRightEncoder();

	}

	public void setPos(double x, double y, double heading){
		this.robotX = x;
		this.robotY = y;
		this.robotHeading = heading;

	}
	public void update(){

		//Update robot values based on encoder and gyro output TODO: plug in actual encoder and gyro values here
//		robotHeading = Gyro.getInstance().getAngle() - calibrateGyroVal;
//		if(robotHeading < 0){
//			robotHeading = robotHeading+360;
//		}
//		robotHeading = 0-robotHeading;
//		double deltaLeftPos = DriveTrain.getInstance().getLeftEncoder() - lastLeftEncoderPos;
//		double deltaRightPos = DriveTrain.getInstance().getRightEncoder() - lastRightEncoderPos;
//		double deltaPosition = (deltaLeftPos + deltaRightPos)/2/TicksPerInch.DRIVE;
//		robotX += deltaPosition * Math.cos(Math.toRadians(robotHeading));
//		robotY += deltaPosition * Math.sin(Math.toRadians(robotHeading));
//		lastLeftEncoderPos = DriveTrain.getInstance().getLeftEncoder();
//		lastRightEncoderPos = DriveTrain.getInstance().getRightEncoder();
//		SmartDashboard.putNumber("robotX", robotX);
//		SmartDashboard.putNumber("robotY", robotY);
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
