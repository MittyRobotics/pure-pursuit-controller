package com.amhsrobotics.purepursuit.simulator;

import com.amhsrobotics.motorsim.graph.RobotGraph;
import com.amhsrobotics.motorsim.simulator.RobotSimManager;
import com.amhsrobotics.motorsim.simulator.SimRobot;
import com.amhsrobotics.motorsim.simulator.SimSampleDrivetrain;
import com.amhsrobotics.purepursuit.PathFollowerPosition;
import com.amhsrobotics.purepursuit.PurePursuitController;
import com.amhsrobotics.purepursuit.PurePursuitOutput;
import com.amhsrobotics.purepursuit.VelocityConstraints;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;

public class PathSimRobot implements SimRobot {

    Path path;
    PurePursuitController controller;
    @Override
    public void robotInit() {
        VelocityConstraints pathVelocityConstraints = new VelocityConstraints(40, 40, 150, 20,30, 0, 2);

        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0,70,0),
                new Coordinate(-50,100,0),
                new Coordinate(-50, 150, 0)
        };
        this.path = new CubicHermitePath(coordinates, pathVelocityConstraints,0,2);
        this.controller = new PurePursuitController(this.path,20,20,false);
        controller.setAdaptiveDistanceGain(.8);
        controller.setkCurvature(1);

        PathFollowerPosition.getInstance().setupRobot(27);
        
        SimSampleDrivetrain.getInstance().getRightMasterTalon().setPIDF(0.01,0,0,.005);
        SimSampleDrivetrain.getInstance().getLeftMasterTalon().setPIDF(0.01,0,0,.005);
    }

    private double t = 0;

    @Override
    public void robotPeriodic() {
        t += RobotSimManager.getInstance().getPeriodTime();
        if(t > 1){
            PathFollowerPosition.getInstance().update(SimSampleDrivetrain.getInstance().getRobotX(), SimSampleDrivetrain.getInstance().getRobotY(), SimSampleDrivetrain.getInstance().getHeading(), SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity(), SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
            PurePursuitOutput output = controller.update(t-1);
    
            System.out.println(output.getLeftVelocity() + " " + output.getRightVelocity());
    
            customTankVelocity(output.getLeftVelocity(), output.getRightVelocity(), output.getAngleToLookahead());
            
//            SimSampleDrivetrain.getInstance().getLeftMasterTalon().setVelocity(output.getLeftVelocity());
//            SimSampleDrivetrain.getInstance().getRightMasterTalon().setVelocity(output.getRightVelocity());
    
            System.out.println(SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity() + " " + SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
        }
    }
    
    private double leftLastMeasured;
    private double rightLastMeasured;
    
    private final double kV = 0.07; //0.07
    private final double kA = 0.0; //0.0
    private final double kP = 0.01; //0.01
    private final double kT = 10;
    
    public void customTankVelocity(double leftVel, double rightVel, double angle){
        double left;
        double right;
        
        angle = CoordinateManager.getInstance().mapAngle(angle);
        angle = (double)Math.round(angle * 100)/100;
        
        
        angle = angle / 45;
        angle = Math.max(-1, Math.min(angle,1));
        
        System.out.println(angle);
        
        double turn = angle * kT;
        
        leftVel += turn;
        rightVel -= turn;
        
        
        double measuredLeft = SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity();
        double FFLeft = kV * leftVel + kA * ((measuredLeft - leftLastMeasured)/.02);
        leftLastMeasured = measuredLeft;
        double errorLeft = leftVel - measuredLeft;
        double FBLeft = kP * errorLeft;
        left = (FFLeft + FBLeft);
        left = Math.max(-12, Math.min(12,left));
        
        double measuredRight = SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity();
        
        double FFRight = kV * rightVel + kA * ((measuredRight - rightLastMeasured)/.02);
        
        rightLastMeasured = measuredRight;
        
        double errorRight = rightVel - measuredRight;
        
        double FBRight = kP * errorRight;
        
        right = (FFRight + FBRight);
        
        right = Math.max(-12, Math.min(12,right));
        
        left = left/12;
        right = right/12;
        
        
        SimSampleDrivetrain.getInstance().setSpeeds(left,right);
    }
}
