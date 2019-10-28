package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.graph.PurePursuitSimulatorGraph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;


import javax.swing.*;

public class Main {



    public static void main(String[] args) {
    
        VelocityConstraints velocityConstraints = new VelocityConstraints(10, 10, 30, 0, 0, 1);
    
        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0, 0, 0),
                new Coordinate(0, 100, 90),
                new Coordinate(20, 100, 90)
        };
    
        Path path = new CubicHermitePath(coordinates, velocityConstraints);
    
        PurePursuitController controller = new PurePursuitController(path);
        PathFollowerPosition.getInstance().update(0, 0, 0);
        
        PurePursuitSimulator simulator = new PurePursuitSimulator(controller);
        simulator.start();
        
    }


}
