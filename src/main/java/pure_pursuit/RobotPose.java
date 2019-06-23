package pure_pursuit;

import java.util.TimerTask;

public class RobotPose extends TimerTask {
	private static RobotPose ourInstance = new RobotPose();

	private double robotX = 0;
	private double robotY = 0;
	private double robotHeading = 0;

	private double lastLeftEncoderPos = 0;
	private double lastRightEncoderPos = 0;

	public static RobotPose getInstance() {
		return ourInstance;
	}

	private RobotPose() {
	}

	public void resetPosition(){
		robotX = 0;
		robotY = 0;
		//lastLeftEncoderPos = getLeftEncoder();
		//lastRightEncoderPos = getRightEncoder();
	}

	public void setPos(double x, double y, double heading){
		this.robotX = x;
		this.robotY = y;
		this.robotHeading = heading;
	}
	public void run(){

		//Update robot values based on encoder and gyro output TODO: plug in actual encoder and gyro values here
		//robotHeading = getGyroAngle();
		//double deltaLeftPos = getLeftEncoder() - lastLeftEncoderPos;
		//double deltaRightPos = getRightEncoder() - lastRightEncoderPos;
		//double deltaPosition = (deltaLeftPos + deltaRightPos)/2;
		//robotX += deltaPosition * cos(robotHeading)
		//robotY += deltaPosition * sin(robotHeading)
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
