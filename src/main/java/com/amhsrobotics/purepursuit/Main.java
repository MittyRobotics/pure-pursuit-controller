package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;

public class Main {
    
    public static void main(String[] args) {
        

        VelocityConstraints pathVelocityConstraints = new VelocityConstraints(40, 30, 150, 20,30, 0, 2);

        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0, 30, 0),
                new Coordinate(-50, 80, 0),
                new Coordinate(-50, 150, 0)
        };
        Path path = new CubicHermitePath(coordinates, pathVelocityConstraints,0,2);

        PurePursuitController controller = new PurePursuitController(path, 30, 20, false);
        PathFollowerPosition.getInstance().update(0, 0, 0,  0, 0);
        PathFollowerPosition.getInstance().setupRobot(27);

        PurePursuitSimulator simulator = new PurePursuitSimulator(controller, 0.02, PathFollowerPosition.getInstance().getTrackWidth());
        simulator.start();
    }
}
