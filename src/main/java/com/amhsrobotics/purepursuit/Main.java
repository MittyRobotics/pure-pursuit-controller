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

    public static void main(String[] args) {
        Graph graph = Graph.getInstance();




        Path path = new CubicHermitePath(new Coordinate[]{new Coordinate(0,0,0),new Coordinate(-100,0,180),new Coordinate(-50,0,90),new Coordinate(-50,-200,180)},new VelocityConstraints(20,20,50,0,0));

//        System.out.println(path.getTrajectoryPoints()[path.getTrajectoryPoints().length-1].getX());

        graph.graphPath(path);
//        graph.graphCircle(0,0,10);

        graph.resizeGraph();

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        graph.resizeGraph();

        PurePursuitController controller = new PurePursuitController(path);
        PathFollowerPosition.getInstance().update(0,0,0);


       while(path.getTrajectoryPoints()[path.getTrajectoryPoints().length-1].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) > 1){

           PurePursuitOutput output = controller.update();


           PathFollowerPosition.getInstance().update(calculateNewRobotPos(output,controller)[0],calculateNewRobotPos(output,controller)[1],calculateNewRobotPos(output,controller)[2]);

           output = controller.update();




            Graph.getInstance().graphCircle(controller.getCurrentCircleCenterPoint().getX(),controller.getCurrentCircleCenterPoint().getY(),controller.getCurrentRadius());

            Graph.getInstance().graphRobot(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY());

            Graph.getInstance().graphTargetPoint(controller.getCurrentTargetPoint().getX(),controller.getCurrentTargetPoint().getY());

            Graph.getInstance().graphVelocity(output.getLeftVelocity(),output.getRightVelocity());


//
//            graph.resizeGraph();

            try {
                Thread.sleep(40);
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
        double x3 = x1 + Math.cos(Math.toRadians(heading))*(output.getRightVelocity());
        double y3 = y1 + Math.sin(Math.toRadians(heading))*(output.getRightVelocity());
        double x4 = x2 + Math.cos(Math.toRadians(heading))*(output.getLeftVelocity());
        double y4 = y2 + Math.sin(Math.toRadians(heading))*(output.getLeftVelocity());

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
