package com.amhsrobotics.purepursuit;
		
		import com.amhsrobotics.purepursuit.coordinate.Position;
		import com.amhsrobotics.purepursuit.coordinate.Rotation;
		import com.amhsrobotics.purepursuit.coordinate.Transform;

public class CoordinateTestMain {
	public static void main(String[] args) {
		Transform t1 = new Transform(new Position(1,0), new Rotation(0));
		Transform t2 = new Transform(new Position(0,0), new Rotation(45));
		
		Transform t = t1.rotateBy(t2).getTransformError(new Transform(new Position(1,1)));
		
		System.out.println(t.toString());
	}
}
