package pure_pursuit;

public class TrajectoryFollowerOutput {
	private double leftVelocity;
	private double rightVelocity;
	public TrajectoryFollowerOutput(double leftVelocity, double rightVelocity){
		this.leftVelocity = leftVelocity;
		this.rightVelocity = rightVelocity;
	}

	public double getLeftVelocity(){
		return leftVelocity;
	}
	public double getRightVelocity(){
		return rightVelocity;
	}

}
