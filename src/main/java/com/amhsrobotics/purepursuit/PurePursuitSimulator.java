package com.amhsrobotics.purepursuit;

import com.amhsrobotics.motorsim.graph.Graph;
import com.amhsrobotics.motorsim.math.Conversions;
import com.amhsrobotics.motorsim.motors.CIMMotor;
import com.amhsrobotics.motorsim.simulator.ControlLoop;
import com.amhsrobotics.motorsim.simulator.ControlLoopType;
import com.amhsrobotics.motorsim.simulator.ControlType;
import com.amhsrobotics.motorsim.simulator.MotorSimulator;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;

import javax.swing.*;
import java.awt.*;

public class PurePursuitSimulator extends Thread {

	
	private double robotWidth;

	private boolean running;
	
	private boolean setup;
	
	private double previousT = 0;
	
	private double t;
	
	private PurePursuitController controller;
	
	private double iterationTime;
	private double loopsPerSecond = 50;
	
	public PurePursuitSimulator(PurePursuitController controller, double iterationTime, double robotWidth){
		this.controller = controller;
		this.running = true;
		this.setup = false;
		this.iterationTime = iterationTime;
		this.loopsPerSecond = 1/iterationTime;
		this.robotWidth = robotWidth;
	}
	
	public void run(){
		
		if(!setup){
			setupGraph();
		}
		
		t =previousT;
		
		double prevVelocity = 1;
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		

		
		double mass = 40.0 * Conversions.LBS_TO_KG;
		double gearRatio = 7.954545454545454;
		double wheelRadius = 2.0 * Conversions.IN_TO_M;
		double maxVoltage = 12.0;
		ControlLoop controlLoop = new ControlLoop(ControlLoopType.VELOCITY, maxVoltage, iterationTime);
		controlLoop.setupVelocityController(3.3, 0.0, 0);
		ControlType controlType = ControlType.VELOCITY;
		MotorSimulator leftMotorSimulator = new MotorSimulator(new CIMMotor(), 2.0D, mass/2, gearRatio, wheelRadius, controlLoop, controlType, "CIM motor");
		MotorSimulator rightMotorSimulator = new MotorSimulator(new CIMMotor(), 2.0D, mass/2, gearRatio, wheelRadius, controlLoop, controlType, "CIM motor");
		
		Graph leftGraph = new Graph("Left CIM Motors");
		Graph rightGraph = new Graph("Right CIM Motors");
		
		leftGraph.setVisible(false);
		rightGraph.setVisible(false);
		
		leftGraph.getContentPane().setPreferredSize(new Dimension(400,400));
		rightGraph.getContentPane().setPreferredSize(new Dimension(400,400));
		
		PurePursuitSimulatorGraph.getInstance().add(leftGraph.getContentPane());
		PurePursuitSimulatorGraph.getInstance().add(rightGraph.getContentPane());
		
		PurePursuitSimulatorGraph.getInstance().pack();
		
		while(prevVelocity != 0 && running){
			
			final PurePursuitOutput output = controller.update(t);
			leftMotorSimulator.update(output.getLeftVelocity() * Conversions.IN_TO_M,iterationTime);
			rightMotorSimulator.update(output.getRightVelocity() * Conversions.IN_TO_M,iterationTime);
			
			leftGraph.addSetpoint(output.getLeftVelocity() * Conversions.IN_TO_M,t);
			leftGraph.addVelocity(leftMotorSimulator.getVelocity(),t);
			leftGraph.addPosition(leftMotorSimulator.getPosition(),t);
			leftGraph.addVoltage(leftMotorSimulator.getVoltage(),t);
			
			rightGraph.addSetpoint(output.getRightVelocity() * Conversions.IN_TO_M,t);
			rightGraph.addVelocity(rightMotorSimulator.getVelocity(),t);
			rightGraph.addPosition(rightMotorSimulator.getPosition(),t);
			rightGraph.addVoltage(rightMotorSimulator.getVoltage(),t);
			
			PurePursuitOutput output1 = new PurePursuitOutput(leftMotorSimulator.getVelocity() * Conversions.M_TO_IN, rightMotorSimulator.getVelocity()* Conversions.M_TO_IN);
			
			PathFollowerPosition.getInstance().update(calculateNewRobotPos(output1)[0],calculateNewRobotPos(output1)[1],calculateNewRobotPos(output1)[2], output1.getLeftVelocity(), output1.getRightVelocity());

			
			final double currentT = t;
			
			double finalPrevVelocity = prevVelocity;
			
			SwingUtilities.invokeLater(() -> {
				PurePursuitSimulatorGraph.getInstance().graphCircle(controller.getCurrentCircleCenterPoint().getX(),controller.getCurrentCircleCenterPoint().getY(),controller.getCurrentRadius());
				
				PurePursuitSimulatorGraph.getInstance().graphRobotPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY(), true);
				
				PurePursuitSimulatorGraph.getInstance().graphTargetPoint(controller.getCurrentTargetPoint().getX(),controller.getCurrentTargetPoint().getY());
				
				PurePursuitSimulatorGraph.getInstance().graphVelocity(output.getLeftVelocity()/loopsPerSecond,output.getRightVelocity()/loopsPerSecond, robotWidth);
				
				PurePursuitSimulatorGraph.getInstance().graphRobotVelocityOverTime((output.getLeftVelocity() + output.getRightVelocity())/2, t);
			});
			
			prevVelocity = (output.getLeftVelocity() + output.getRightVelocity())/2;
			
			t += iterationTime;

			if(Math.abs(prevVelocity) <  1){
				break;
			}

			try {
				Thread.sleep((long) (1000/loopsPerSecond));
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		System.out.println("test");
		stopSimulator();
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
	

	public double[] calculateNewRobotPos(PurePursuitOutput output){
		double heading = PathFollowerPosition.getInstance().getPathCentricHeading();
		double x1 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading-90))*(robotWidth/2);
		double y1 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading-90))*(robotWidth/2);
		double x2 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading+90))*(robotWidth/2);
		double y2 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading+90))*(robotWidth/2);
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
