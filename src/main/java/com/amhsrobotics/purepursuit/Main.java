package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;


import javax.swing.*;

public class Main {
    
    public static void main(String[] args) {
    
        VelocityConstraints velocityConstraints = new VelocityConstraints(20, 20, 20, 0, 0, 1);
    
        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0, 45, 0),
                new Coordinate(5, 50, 90),
                new Coordinate(45, 50, 90),
                new Coordinate(50, 55, 0),
                new Coordinate(50, 100, 0)
        };
    
        Path path = new CubicHermitePath(coordinates, velocityConstraints);
        
        
        PurePursuitController controller = new PurePursuitController(path,15,5);
        PathFollowerPosition.getInstance().update(0, 0, 0);
        PathFollowerPosition.getInstance().setupRobot(20);
        
        PurePursuitSimulator simulator = new PurePursuitSimulator(controller,60,PathFollowerPosition.getInstance().getTrackWidth());
        simulator.start();
        
    }


}
