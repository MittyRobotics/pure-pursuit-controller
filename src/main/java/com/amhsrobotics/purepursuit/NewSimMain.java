package com.amhsrobotics.purepursuit;

import com.amhsrobotics.motorsim.simulator.RobotSimManager;
import com.amhsrobotics.motorsim.simulator.SimOI;
import com.amhsrobotics.motorsim.simulator.SimSampleDrivetrain;
import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.simulator.PathSimRobot;

public class NewSimMain {
    public static void main(String[] args) {
        PathSimRobot robot = new PathSimRobot();
        RobotSimManager.getInstance().setupRobotSimManager(robot, SimSampleDrivetrain.getInstance(),125,7,2,20,30,0.02);


    }
}
