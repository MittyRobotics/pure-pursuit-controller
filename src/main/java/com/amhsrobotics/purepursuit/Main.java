package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;

public class Main {
    
    public static void main(String[] args) {

        VelocityConstraints pathVelocityConstraints = new VelocityConstraints(40, 40, 100, 0, 0, 1);
        VelocityConstraints wheelVelocityConstraints = new VelocityConstraints(10000, 10000, 150);

        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0, 45, 0),
                new Coordinate(5, 50, 90),
                new Coordinate(45, 50, 90),
                new Coordinate(50, 55, 0),
                new Coordinate(50, 100, 0)
        };
        Path path = new CubicHermitePath(coordinates, pathVelocityConstraints);

        PurePursuitController controller = new PurePursuitController(path, 20, 10, wheelVelocityConstraints, false);
        PathFollowerPosition.getInstance().update(0, 0, 0,  0, 0);
        PathFollowerPosition.getInstance().setupRobot(27);
//
//        PurePursuitSimulator simulator = new PurePursuitSimulator(controller, 0.02, PathFollowerPosition.getInstance().getTrackWidth());
//        simulator.start();
    }
}
