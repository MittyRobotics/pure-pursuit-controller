package com.amhsrobotics.purepursuit.coordinate;

/**
 * A Transform object that holds the robot's {@link Position} and {@link Rotation} object, making up the robot's overall
 * pose. Unlike the WPILib's Transform2d object, which is used for transformations of Pose2d objects, this Transform
 * object is used to store the robot's actual pose instead of a pose transformation.
 */
public class Transform {
	private Position position;
	private Rotation rotation;
	
	public Transform(){
		this(new Position(), new Rotation());
	}
	
	public Transform(Position position){
		this(position, new Rotation());
	}
	
	public Transform(Rotation rotation){
		this(new Position(), rotation);
	}
	
	public Transform(double x, double y, double heading){
		this(new Position(x,y), new Rotation(heading));
	}
	
	public Transform(double x, double y, Rotation rotation){
		this(new Position(x,y),rotation);
	}
	
	public Transform(Position position, double heading){
		this(position, new Rotation(heading));
	}
	
	public Transform(Position position, Rotation rotation){
		this.position = position;
		this.rotation = rotation;
	}
	
	public Position getPosition() {
		return position;
	}
	
	public Rotation getRotation() {
		return rotation;
	}
	
	public Transform multiply(double scalar){
		Position pos = position.multiply(scalar);
		Rotation rot = rotation.multiply(scalar);
		
		return new Transform(pos,rot);
	}
	
	
	public Transform divide(double scalar){
		Position pos = position.divide(scalar);
		Rotation rot = rotation.divide(scalar);
		
		return new Transform(pos,rot);
	}
	
	public Transform add(Transform other){
		Position pos = position.add(other.getPosition());
		Rotation rot = rotation.add(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public Transform subtract(Transform other){
		Position pos = position.subtract(other.getPosition());
		Rotation rot = rotation.subtract(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public Transform rotateBy(Transform other){
		Position pos = position.rotateBy(other.getRotation());
		Rotation rot = rotation.rotateBy(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public Transform getTransformError(Transform other){
		Position pos = position.subtract(other.getPosition());
		Rotation rot = rotation.subtract(other.getRotation());
		
		return new Transform(pos,rot);
	}
	
	public void setPosition(Position position) {
		this.position = position;
	}
	
	public void setRotation(Rotation rotation) {
		this.rotation = rotation;
	}
	
	@Override
	public String toString() {
		return String.format("Transform(%s, %s)", position.toString(), rotation.toString());
	}
	
}
