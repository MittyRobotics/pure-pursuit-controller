package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.graph.Graph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;

import javax.swing.*;
import java.awt.*;

public class Main {
    public static void main(String[] args) {
//
        Graph graph = Graph.getInstance();
        graph.pack();
        graph.setVisible(true);

        TestGraph testGraph = new TestGraph();

        CoordinateSystem system = new CoordinateSystem(90, TurnSign.POSITIVE, VectorDirection.NEGATIVE_Y, VectorDirection.NEGATIVE_X);
//
//
//        testGraph.panel2.setLocation(100, 200);
//
//
//        testGraph.panel4.setLocation(300, 200);


//        for (int i = 0; i < 360*3; i++) {
//
//           double angle = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,CoordinateManager.getInstance().mapAngle(i)),system).getAngle();
//            double angle1 = CoordinateManager.getInstance().mapAngle(i);
//            System.out.println(angle + " " + angle1);
//            testGraph.panel1.setLocation((int) (100 + Math.cos(Math.toRadians(angle)) * 100), (int) (200 + Math.sin(Math.toRadians(angle)) * 100));
//
//            testGraph.panel3.setLocation((int) (300 + Math.cos(Math.toRadians(angle1)) * 100), (int) (200 + Math.sin(Math.toRadians(angle1)) * 100));
//
//            testGraph.repaint();
//            testGraph.revalidate();
//
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }

        Path path = new CubicHermitePath(new Coordinate[]{new Coordinate(0,0,0),new Coordinate(0,100,0)},new VelocityConstraints(20,20,50,0,0));

        graph.graphPath(path);
        graph.graphCircle(0,0,10);

        graph.resizeGraph();

        PathFollowerPosition.getInstance().update(0,0,90);

        PurePursuitController controller = new PurePursuitController(path);
        controller.update();
        //Graph.getInstance().graphCircle(PathFollowerPosition.getInstance().getX(),PathFollowerPosition.getInstance().getY(),10);
        Graph.getInstance().graphCircle(controller.getCurrentCircleCenterPoint().getX(),controller.getCurrentCircleCenterPoint().getY(),controller.getCurrentRadius());

        System.out.println(controller.getCurrentTargetPoint().getX() + " " + controller.getCurrentTargetPoint().getY()+ " " + controller.getCurrentRadius());
        for (int i = 0; i < path.getTrajectoryPoints().length; i++) {

            JPanel panel = new JPanel();
            panel.setSize(new Dimension(10,10));
            panel.setBackground(Color.BLACK);

            panel.setLocation((int) (200 +  path.getTrajectoryPoints()[i].getX()),(int) (200 - path.getTrajectoryPoints()[i].getY()));
            testGraph.add(panel);


            testGraph.repaint();
            testGraph.revalidate();

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


//        for (int i = 0; i < 200; i++) {
//
//            Coordinate inputCoord = new Coordinate(i,i,0);
//            Coordinate newCoord = CoordinateManager.getInstance().coordinateTransformation(inputCoord,system);
//
//            testGraph.panel1.setLocation((int) (100 + newCoord.getX()),(int) (400 + newCoord.getY()));
//
//            testGraph.panel3.setLocation((int) (300 + inputCoord.getX()),(int) (400 + inputCoord.getY()));
//
//            testGraph.repaint();
//            testGraph.revalidate();
//
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
//
//        Coordinate testCoord = new Coordinate(10,0,0);
//
//
//        System.out.println(CoordinateManager.getInstance().coordinateTransformation(testCoord, system).getX() + " " + CoordinateManager.getInstance().coordinateTransformation(testCoord, system).getY() );
    }
}
