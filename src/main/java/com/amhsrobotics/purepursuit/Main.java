package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.*;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.graph.Graph;
import com.amhsrobotics.purepursuit.paths.CubicHermitePath;
import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.paths.TrajectoryPoint;


import javax.swing.*;
import java.awt.*;
import java.awt.geom.Point2D;

public class Main {

    public static double LOOPS_PER_SECOND = 60;

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {

        });

        Graph graph = Graph.getInstance();


        VelocityConstraints velocityConstraints = new VelocityConstraints(10,10,30,0,0,1);
        Coordinate[] coordinates = new Coordinate[]{
                new Coordinate(0,0,0),
                new Coordinate(0,100,90),
                new Coordinate(20,100,90)
        };

        Path path = new CubicHermitePath(coordinates,velocityConstraints);

//        System.out.println(path.getTrajectoryPoints()[path.getTrajectoryPoints().length-1].getX());

        graph.graphPath(path);

        graph.resizeGraph();


        PurePursuitController controller = new PurePursuitController(path);
        PathFollowerPosition.getInstance().update(0,0,0);

        Graph.getInstance().graphPathVelocity(path);


        double t =0;

        double prevVelocity = 1;

        while(prevVelocity != 0){


            final PurePursuitOutput output = controller.update(t / 1000);

            PathFollowerPosition.getInstance().update(calculateNewRobotPos(output,controller)[0],calculateNewRobotPos(output,controller)[1],calculateNewRobotPos(output,controller)[2]);


            final PurePursuitOutput output1 = controller.update(t / 1000);

            final double currentT = t;

            double finalPrevVelocity = prevVelocity;
            SwingUtilities.invokeLater(() -> {
                Graph.getInstance().graphCircle(controller.getCurrentCircleCenterPoint().getX(),controller.getCurrentCircleCenterPoint().getY(),controller.getCurrentRadius());

                Graph.getInstance().graphRobot(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY());

                Graph.getInstance().graphTargetPoint(controller.getCurrentTargetPoint().getX(),controller.getCurrentTargetPoint().getY());

                Graph.getInstance().graphVelocity(output1.getLeftVelocity()/LOOPS_PER_SECOND,output1.getRightVelocity()/LOOPS_PER_SECOND);

                Graph.getInstance().graphRobotVelocityOverTime((output1.getLeftVelocity() + output1.getRightVelocity())/2, currentT/1000);
            });

            prevVelocity = (output1.getLeftVelocity() + output1.getRightVelocity())/2;

            t += (long)1000/(long)LOOPS_PER_SECOND;

            try {
                Thread.sleep((long)1000/(long)LOOPS_PER_SECOND);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }



    }

    public static double[] calculateNewRobotPos(PurePursuitOutput output, PurePursuitController controller){
        double heading = PathFollowerPosition.getInstance().getPathCentricHeading();
        double x1 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading-90))*10;
        double y1 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading-90))*10;
        double x2 = PathFollowerPosition.getInstance().getPathCentricX() + Math.cos(Math.toRadians(heading+90))*10;
        double y2 = PathFollowerPosition.getInstance().getPathCentricY() + Math.sin(Math.toRadians(heading+90))*10;
        double x3 = x1 + Math.cos(Math.toRadians(heading))*(output.getRightVelocity())/LOOPS_PER_SECOND;
        double y3 = y1 + Math.sin(Math.toRadians(heading))*(output.getRightVelocity())/LOOPS_PER_SECOND;
        double x4 = x2 + Math.cos(Math.toRadians(heading))*(output.getLeftVelocity())/LOOPS_PER_SECOND;
        double y4 = y2 + Math.sin(Math.toRadians(heading))*(output.getLeftVelocity())/LOOPS_PER_SECOND;

        double x = (x3+x4)/2;
        double y = (y3+y4)/2;

        double a = -Math.toDegrees(Math.atan2((y4-y3),(x4-x3))) ;

        //System.out.println(a + " " + (y4-y3) + " " + (x4-x3));

        CoordinateSystem system = new CoordinateSystem(180, TurnSign.NEGATIVE, VectorDirection.NEGATIVE_Y, VectorDirection.NEGATIVE_X);

        Coordinate coordinate = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,a),system);

        a = coordinate.getAngle();

        return new double[]{x,y,a};
    }
}
