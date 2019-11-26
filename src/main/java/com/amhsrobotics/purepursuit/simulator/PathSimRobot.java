package com.amhsrobotics.purepursuit.simulator;

import com.amhsrobotics.motorsim.graph.Graph;
import com.amhsrobotics.motorsim.graph.RobotGraph;
import com.amhsrobotics.motorsim.math.Conversions;
import com.amhsrobotics.motorsim.simulator.RobotSimManager;
import com.amhsrobotics.motorsim.simulator.SimRobot;
import com.amhsrobotics.motorsim.simulator.SimSampleDrivetrain;
import com.amhsrobotics.purepursuit.*;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;

import javax.print.attribute.standard.PresentationDirection;
import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowEvent;

public class PathSimRobot implements SimRobot {
	
	Path path;
	PurePursuitController controller;
	
	private Graph leftGraph;
	private Graph rightGraph;
	
	@Override
	public void robotInit() {
		VelocityConstraints pathVelocityConstraints = new VelocityConstraints(40, 40, 150, 20, 30, 0, 2);
		
		Coordinate[] coordinates = new Coordinate[]{
				new Coordinate(0, 0, 0),
				new Coordinate(-30, 50, 0),
				new Coordinate(-30, 110, 0),
				new Coordinate(-10, 150, 45),
		};
		this.path = new CubicHermitePath(coordinates, pathVelocityConstraints, 0, 2);
		this.controller = new PurePursuitController(this.path, 15, 20, false);
		controller.setAdaptiveDistanceGain(.8);
		controller.setkCurvature(2);
		
		PathFollowerPosition.getInstance().setupRobot(27);
		SimSampleDrivetrain.getInstance().setOdometry(0, 0, 0);
		
		SimSampleDrivetrain.getInstance().getRightMasterTalon().setPIDF(0.0, 0, 0, .005);
		SimSampleDrivetrain.getInstance().getLeftMasterTalon().setPIDF(0.0, 0, 0, .005);
        
        
        setupGraph();
        
    }
	
	private double t = 0;
	
	@Override
	public void robotPeriodic() {
		
		t += RobotSimManager.getInstance().getPeriodTime();
		
		PathFollowerPosition.getInstance().update(SimSampleDrivetrain.getInstance().getRobotX(), SimSampleDrivetrain.getInstance().getRobotY(), SimSampleDrivetrain.getInstance().getHeading(), SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity(), SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
		PurePursuitOutput output = controller.update(t - 1);
		
		//System.out.println(output.getLeftVelocity() + " " + output.getRightVelocity());
		
		double radius = 1/controller.getCurrentClosestPoint().getRawCurvature();
		
		//output = ramseteTest(controller.getCurrentClosestPoint().getVelocity(), controller.getCurrentClosestPoint().getVelocity()/radius,controller.getCurrentClosestPoint().getX(), controller.getCurrentClosestPoint().getY(),Math.toRadians(controller.getCurrentClosestPoint().getAngle()));
		
		
		customTankVelocity(output.getLeftVelocity(), output.getRightVelocity(), output.getAngleToLookahead());
		
		//System.out.println(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity() + " " + SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
		graph(output);
		
//		if(t > 5){
//		    t = 0;
//            SimSampleDrivetrain.getInstance().setOdometry(0,0,0);
//            this.controller = new PurePursuitController(this.path, 20, 20, false);
//            controller.setAdaptiveDistanceGain(.8);
//            controller.setkCurvature(2);
//            PurePursuitSimulatorGraph.getInstance().dispose();
//            PurePursuitSimulatorGraph.setInstance(new PurePursuitSimulatorGraph());
//            setupGraph();
//        }

	}
	
	private void setupGraph(){
        Graph leftGraph = new Graph("Left CIM Motors");
        Graph rightGraph = new Graph("Right CIM Motors");
        
        leftGraph.setVisible(false);
        rightGraph.setVisible(false);
        
        leftGraph.getContentPane().setPreferredSize(new Dimension(200,200));
        rightGraph.getContentPane().setPreferredSize(new Dimension(200,200));
        
        PurePursuitSimulatorGraph.getInstance().add(leftGraph.getContentPane());
        PurePursuitSimulatorGraph.getInstance().add(rightGraph.getContentPane());
        RobotGraph.getInstance().getContentPane().setPreferredSize(new Dimension(600,600));
        
        PurePursuitSimulatorGraph.getInstance().add(RobotGraph.getInstance().getContentPane());
        
        JFrame placeHolderFrame = new JFrame();
        placeHolderFrame.setBackground(new Color(71, 71, 71));
        placeHolderFrame.getContentPane().setBackground(new Color(71,71,71));
        
        PurePursuitSimulatorGraph.getInstance().add(placeHolderFrame.getContentPane());
        
        RobotGraph.getInstance().setVisible(false);
        
        this.leftGraph = leftGraph;
        this.rightGraph = rightGraph;
        
        PurePursuitSimulatorGraph.getInstance().pack();
        
        PurePursuitSimulatorGraph.getInstance().graphPathVelocity(controller.getPath());
        
        PurePursuitSimulatorGraph.getInstance().graphPath(controller.getPath());
        
        PurePursuitSimulatorGraph.getInstance().resizeGraph();
        
 }
	
	private void graph(PurePursuitOutput output){
        SwingUtilities.invokeLater(() -> {
            leftGraph.addSetpoint(output.getLeftVelocity() * Conversions.IN_TO_M, t);
            leftGraph.addVelocity(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity() * Conversions.IN_TO_M, t);
            leftGraph.addPosition(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getPosition() * Conversions.IN_TO_M, t);
            leftGraph.addVoltage(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVoltage(), t);
    
            rightGraph.addSetpoint(output.getRightVelocity() * Conversions.IN_TO_M, t);
            rightGraph.addVelocity(SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity() * Conversions.IN_TO_M, t);
            rightGraph.addPosition(SimSampleDrivetrain.getInstance().getRightMasterTalon().getPosition() * Conversions.IN_TO_M, t);
            rightGraph.addVoltage(SimSampleDrivetrain.getInstance().getRightMasterTalon().getVoltage(), t);
            
            PurePursuitSimulatorGraph.getInstance().graphCircle(controller.getCurrentCircleCenterPoint().getX(), controller.getCurrentCircleCenterPoint().getY(), controller.getCurrentRadius());
            
            PurePursuitSimulatorGraph.getInstance().graphRobotPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY(), true);
            
            PurePursuitSimulatorGraph.getInstance().graphTargetPoint(controller.getCurrentTargetPoint().getX(), controller.getCurrentTargetPoint().getY());
            
            PurePursuitSimulatorGraph.getInstance().graphVelocity(output.getLeftVelocity()/10  , output.getRightVelocity()/10 , RobotSimManager.getInstance().getRobotWidth());
            PurePursuitSimulatorGraph.getInstance().graphRobotVelocityOverTime((output.getLeftVelocity() + output.getRightVelocity()) / 2, t);
        });
    }
    
    private PurePursuitOutput ramseteTest(double desiredVelocity, double desiredTurningVelocity, double desiredX, double desiredY, double desiredAngle){
		double x = PathFollowerPosition.getInstance().getX();
		double y = PathFollowerPosition.getInstance().getY();
		double angle = -Math.toRadians(PathFollowerPosition.getInstance().getHeading());
	    
		double xDiff = desiredX-x;
		double yDiff = desiredY-y;
		double angleDiff = desiredAngle-angle;
		
		double eY = Math.cos(angle) * xDiff + Math.sin(angle) * yDiff;
		double eX = -Math.sin(angle) * xDiff + Math.cos(angle) * yDiff;
		double eA = angleDiff;
		
		double beta = 1.0;// > 0, makes convergence more aggressive
		double zeta = 0.1; //[0,1], provides more damping
	 
	    double k = 2*zeta*Math.sqrt(desiredTurningVelocity*desiredTurningVelocity + beta*(desiredVelocity*desiredVelocity));
	    
	    double linearVelocity = desiredVelocity*Math.cos(eA)  + k * eX;
	    linearVelocity = linearVelocity;
	    
	    double angularVelocity = desiredTurningVelocity + k*eA /* + beta*desiredVelocity*(Math.sin(eA)/eA)*eY*/;
		angularVelocity = -angularVelocity;
	
	    PurePursuitOutput output = solveInverseKinematics(linearVelocity,angularVelocity);
	
	
	    System.out.println("Linear: " + linearVelocity + " Angluar: " + angularVelocity +  " Left: " + output.getLeftVelocity() + " Right: " + output.getRightVelocity() + " eX: " + eX + " eY" + eY + " eA: " + eA + " desiredTurning: " + desiredTurningVelocity + " Desired Angle: " + desiredAngle + " k: " + k + " angle: " + angle);

		return output;
    }
	

    
    private PurePursuitOutput solveInverseKinematics(double linearVelocity, double angularVelocity){
		double vR = ((2*linearVelocity) + (angularVelocity*RobotSimManager.getInstance().getRobotWidth()))/(2*RobotSimManager.getInstance().getDriveWheelRadius());
		double vL = ((2*linearVelocity) - (angularVelocity*RobotSimManager.getInstance().getRobotWidth()))/(2*RobotSimManager.getInstance().getDriveWheelRadius());
		return new PurePursuitOutput(vR,vL,0);
    }
    
	
	private double leftLastMeasured;
	private double rightLastMeasured;
	
	private final double kV = 0.08; //0.07
	private final double kA = 0.0; //0.0
	private final double kP = 0.01; //0.01
	private final double kT = 0;
	
	public void customTankVelocity(double leftVel, double rightVel, double angle) {
		double left;
		double right;
		
		angle = CoordinateManager.getInstance().mapAngle(angle);
		angle = (double) Math.round(angle * 100) / 100;
		
		
		angle = angle / 45;
		angle = Math.max(-1, Math.min(angle, 1));
		
		System.out.println(angle);
		
		double turn = angle * kT;
		
		if ((leftVel + rightVel) / 2 < 0) {
			leftVel -= turn;
			rightVel += turn;
		} else {
			leftVel += turn;
			rightVel -= turn;
		}
		
		
		double measuredLeft = SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity();
		double FFLeft = kV * leftVel + kA * ((measuredLeft - leftLastMeasured) / RobotSimManager.getInstance().getPeriodTime());
		leftLastMeasured = measuredLeft;
		double errorLeft = leftVel - measuredLeft;
		double FBLeft = kP * errorLeft;
		left = (FFLeft + FBLeft);
		left = Math.max(-12, Math.min(12, left));
		
		double measuredRight = SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity();
		
		double FFRight = kV * rightVel + kA * ((measuredRight - rightLastMeasured) / RobotSimManager.getInstance().getPeriodTime());
		
		rightLastMeasured = measuredRight;
		
		double errorRight = rightVel - measuredRight;
		
		double FBRight = kP * errorRight;
		
		right = (FFRight + FBRight);
		
		right = Math.max(-12, Math.min(12, right));
		
		left = left / 12;
		right = right / 12;
		
		
		SimSampleDrivetrain.getInstance().setSpeeds(left, right);
	}
}
