package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;


import javax.swing.*;

public class Main {
    
    public static void main(String[] args) {
    
        VelocityConstraints velocityConstraints = new VelocityConstraints(20, 10, 50, 0, 0, 1);
    
        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0, 85, 0),
                new Coordinate(30, 90, 90),
                new Coordinate(30, 50, 90),
                new Coordinate(50, 55, 0),
                new Coordinate(50, 100, 0)
        };
        Path path = new CubicHermitePath(coordinates, velocityConstraints);

        PurePursuitController controller = new PurePursuitController(path,15,10);
        PathFollowerPosition.getInstance().update(0, 0, 0);
        PathFollowerPosition.getInstance().setupRobot(27);
        
        PurePursuitSimulator simulator = new PurePursuitSimulator(controller,60,PathFollowerPosition.getInstance().getTrackWidth());
        simulator.start();


    }


}
