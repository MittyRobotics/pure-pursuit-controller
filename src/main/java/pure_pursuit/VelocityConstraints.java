package pure_pursuit;

public class VelocityConstraints {
	private double maxAcceleration;
	private double maxVelocity;
	public VelocityConstraints(double maxAcceleration, double maxVelocity){
		this.maxAcceleration = maxAcceleration;
		this.maxVelocity = maxVelocity;
	}
	public double getMaxAcceleration(){
		return maxAcceleration;
	}
	public double getMaxVelocity(){
		return maxVelocity;
	}
}
