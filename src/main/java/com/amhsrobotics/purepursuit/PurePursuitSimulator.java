package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;

import javax.swing.*;
import java.awt.event.WindowEvent;

public class PurePursuitSimulator extends Thread {
	private double loopsPerSecond = 60;
	

	private boolean running;
	
	private boolean setup;
	
	private double previousT = 0;
	
	private double t;
	
	private PurePursuitController controller;
	
	public PurePursuitSimulator(PurePursuitController controller){
		this.controller = controller;
		running = true;
		setup = false;
	}
	
	public void run(){
		
		if(!setup){
			setupGraph();
		}

		
		
		t =previousT;
		
		double prevVelocity = 1;
		
		while(prevVelocity != 0 && running){
			
			
			final PurePursuitOutput output = controller.update(t / 1000);
			
			PathFollowerPosition.getInstance().update(calculateNewRobotPos(output,controller)[0],calculateNewRobotPos(output,controller)[1],calculateNewRobotPos(output,controller)[2]);
			
			
			final PurePursuitOutput output1 = controller.update(t / 1000);
			
			final double currentT = t;
			
			double finalPrevVelocity = prevVelocity;
			SwingUtilities.invokeLater(() -> {
				PurePursuitSimulatorGraph.getInstance().graphCircle(controller.getCurrentCircleCenterPoint().getX(),controller.getCurrentCircleCenterPoint().getY(),controller.getCurrentRadius());
				
				PurePursuitSimulatorGraph.getInstance().graphRobot(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY());
				
				PurePursuitSimulatorGraph.getInstance().graphTargetPoint(controller.getCurrentTargetPoint().getX(),controller.getCurrentTargetPoint().getY());
				
				PurePursuitSimulatorGraph.getInstance().graphVelocity(output1.getLeftVelocity()/loopsPerSecond,output1.getRightVelocity()/loopsPerSecond);
				
				PurePursuitSimulatorGraph.getInstance().graphRobotVelocityOverTime((output1.getLeftVelocity() + output1.getRightVelocity())/2, currentT/1000);
			});
			
			prevVelocity = (output1.getLeftVelocity() + output1.getRightVelocity())/2;
			
			t += 1000/loopsPerSecond;
			
			try {
				Thread.sleep((long)1000/(long)loopsPerSecond);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
	
	
	
	public void stopSimulator(){
		running = false;
		previousT = t;
	}
	
	public void resumeSimulator(){
		running = true;
		run();
	}
	
	private void setupGraph(){
		
		PurePursuitSimulatorGraph.getInstance().graphPathVelocity(controller.getPath());
		
		PurePursuitSimulatorGraph.getInstance().graphPath(controller.getPath());
		
		PurePursuitSimulatorGraph.getInstance().resizeGraph();
		
		setup = true;
	}
	

	public double[] calculateNewRobotPos(PurePursuitOutput output, PurePursuitController controller){
		double heading = PathFollowerPosition.getInstance().getPathCentricHeading();
		double x1 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading-90))*10;
		double y1 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading-90))*10;
		double x2 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading+90))*10;
		double y2 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading+90))*10;
		double x3 = x1 + Math.cos(Math.toRadians(heading))*(output.getRightVelocity())/loopsPerSecond;
		double y3 = y1 + Math.sin(Math.toRadians(heading))*(output.getRightVelocity())/loopsPerSecond;
		double x4 = x2 + Math.cos(Math.toRadians(heading))*(output.getLeftVelocity())/loopsPerSecond;
		double y4 = y2 + Math.sin(Math.toRadians(heading))*(output.getLeftVelocity())/loopsPerSecond;
		
		double x = (x3+x4)/2;
		double y = (y3+y4)/2;
		
		double a = -Math.toDegrees(Math.atan2((y4-y3),(x4-x3))) ;
		
		//System.out.println(a + " " + (y4-y3) + " " + (x4-x3));
		
		CoordinateSystem system = new CoordinateSystem(180, TurnSign.NEGATIVE, VectorDirection.NEGATIVE_Y, VectorDirection.NEGATIVE_X);
		
		Coordinate coordinate = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,a),system);
		
		a = coordinate.getAngle();
		
		return new double[]{x,y,a};
	}
	
	public double getLoopsPerSecond() {
		return loopsPerSecond;
	}
	
	public void setLoopsPerSecond(double loopsPerSecond) {
		this.loopsPerSecond = loopsPerSecond;
	}
	public boolean isRunning() {
		return running;
	}
	
	public void setRunning(boolean running) {
		this.running = running;
	}
	
	public PurePursuitController getController() {
		return controller;
	}
	
	public void setController(PurePursuitController controller) {
		this.controller = controller;
	}
	
	
	
}
