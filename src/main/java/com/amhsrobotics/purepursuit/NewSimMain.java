package com.amhsrobotics.purepursuit;

import com.amhsrobotics.motorsim.graph.RobotGraph;
import com.amhsrobotics.motorsim.simulator.RobotSimManager;
import com.amhsrobotics.motorsim.simulator.SimOI;
import com.amhsrobotics.motorsim.simulator.SimSampleDrivetrain;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.simulator.PathSimRobot;

import java.awt.*;

public class NewSimMain {
    
    public static void main(String[] args) {
    
        PathSimRobot robot = new PathSimRobot();
        RobotSimManager.getInstance().setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(),125,7,2,27,40,0.02);
        PathFollowerPosition.getInstance().update(0,0,0,0,0);
        new Thread(RobotSimManager.getInstance()).start();
        RobotGraph.getInstance().resizeGraph(-20,200,-200,20);
    }
}
