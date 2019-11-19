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
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;

public class PathSimRobot implements SimRobot {

    Path path;
    PurePursuitController controller;
    @Override
    public void robotInit() {
        VelocityConstraints pathVelocityConstraints = new VelocityConstraints(40, 30, 150, 20,30, 0, 2);

        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0, 30, 0),
                new Coordinate(-50, 80, 0),
                new Coordinate(-50, 150, 0)
        };
        this.path = new CubicHermitePath(coordinates, pathVelocityConstraints,20,2);
        this.controller = new PurePursuitController(this.path,20,20,false);
        controller.setAdaptiveDistanceGain(.8);
        controller.setkCurvature(.9);

        PathFollowerPosition.getInstance().setupRobot(20);
    }

    double t = 0;

    @Override
    public void robotPeriodic() {
        t += RobotSimManager.getInstance().getPeriodTime();
        PathFollowerPosition.getInstance().update(SimSampleDrivetrain.getInstance().getRobotX(), SimSampleDrivetrain.getInstance().getRobotY(), SimSampleDrivetrain.getInstance().getHeading(), SimSampleDrivetrain.getInstance().getLeftMasterTalon().getVelocity(), SimSampleDrivetrain.getInstance().getRightMasterTalon().getVelocity());
        PurePursuitOutput output = controller.update(t);

        SimSampleDrivetrain.getInstance().getLeftMasterTalon().setVelocity(output.getLeftVelocity());
        SimSampleDrivetrain.getInstance().getRightMasterTalon().setVelocity(output.getRightVelocity());
        System.out.println("asdf");
        //RobotGraph.getInstance().graphRobot(SimSampleDrivetrain.getInstance().getRobotX(), SimSampleDrivetrain.getInstance().getRobotY(), SimSampleDrivetrain.getInstance().getHeading(),20,30);
    }
}
