package com.amhsrobotics.purepursuit;

import java.awt.*;

public class Main {
    public static void main(String[] args) {
        TestGraph graph = new TestGraph();

        CoordinateSystem system = new CoordinateSystem(90,TurnSign.POSITIVE, VectorDirection.NEGATIVE_Y, VectorDirection.NEGATIVE_X);


        graph.panel2.setLocation(100, 200);


        graph.panel4.setLocation(300, 200);


//        for (int i = 0; i < 360*3; i++) {
//
//           double angle = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,CoordinateManager.getInstance().mapAngle(i)),system).getAngle();
//            double angle1 = CoordinateManager.getInstance().mapAngle(i);
//            System.out.println(angle + " " + angle1);
//            graph.panel1.setLocation((int) (100 + Math.cos(Math.toRadians(angle)) * 100), (int) (200 + Math.sin(Math.toRadians(angle)) * 100));
//
//            graph.panel3.setLocation((int) (300 + Math.cos(Math.toRadians(angle1)) * 100), (int) (200 + Math.sin(Math.toRadians(angle1)) * 100));
//
//            graph.repaint();
//            graph.revalidate();
//
//            try {
//                Thread.sleep(20);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }

        for (int i = 0; i < 200; i++) {

            Coordinate inputCoord = new Coordinate(i,i,0);
            Coordinate newCoord = CoordinateManager.getInstance().coordinateTransformation(inputCoord,system);

            graph.panel1.setLocation((int) (100 + newCoord.getX()),(int) (400 + newCoord.getY()));

            graph.panel3.setLocation((int) (300 + inputCoord.getX()),(int) (400 + inputCoord.getY()));

            graph.repaint();
            graph.revalidate();

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        Coordinate testCoord = new Coordinate(10,0,0);


        System.out.println(CoordinateManager.getInstance().coordinateTransformation(testCoord, system).getX() + " " + CoordinateManager.getInstance().coordinateTransformation(testCoord, system).getY() );
    }
}
