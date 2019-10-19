package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.graph.Graph;
import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.paths.TrajectoryPoint;

import java.awt.*;
import java.awt.geom.Point2D;

/**
 * Pure Pursuit Controller class
 * <p>
 * References: http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 * http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 */
public class PurePursuitController {

    private double lookaheadDistance = 15;
    private double trackWidth = 20;
    private Path path;

    private double currentRadius;



    private TrajectoryPoint currentClosestPoint;
    private TrajectoryPoint currentTargetPoint;
    private Point2D.Double currentCircleCenterPoint;
    private int prevTargetIndex = 0;

    public PurePursuitController(Path path) {
        this.path = path;
    }

    public PurePursuitOutput update() {
        calculateTargetPoint();
        calculateRadiusToTarget();
        return new PurePursuitOutput(leftVelocityFromRadius(), rightVelocityFromRadius());
    }

    public double leftVelocityFromRadius() {
        double baseVelocity = currentClosestPoint.getVelocity();

        double angularVelocity = baseVelocity / getCurrentRadius();

        return angularVelocity * (getCurrentRadius() - (trackWidth / 2));
    }

    public double rightVelocityFromRadius() {
        double baseVelocity = currentClosestPoint.getVelocity();

        double angularVelocity = baseVelocity / getCurrentRadius();

        return angularVelocity * (getCurrentRadius() + (trackWidth / 2));
    }

    public void calculateRadiusToTarget() {

        final double robotAngle = PathFollowerPosition.getInstance().getPathCentricHeading();

        final Point2D.Double vectorHead = new Point2D.Double(Math.cos(Math.toRadians(robotAngle)), Math.sin(Math.toRadians(robotAngle)));

        final double a = this.currentTargetPoint.getX()-PathFollowerPosition.getInstance().getX();
        final double b = this.currentTargetPoint.getY()-PathFollowerPosition.getInstance().getY();
        final double c = vectorHead.getX();
        final double d = vectorHead.getY();

        final double v = 2 * ((a * d) - (b * c));
        final double x = (d * ((a * a) + (b * b))) / v;
        final double y = -(c * (a * a + b * b)) / v;


        final Point2D.Double circleCenter = new Point2D.Double(x, y);

        currentCircleCenterPoint = new Point2D.Double(circleCenter.getX() + PathFollowerPosition.getInstance().getPathCentricX(), circleCenter.getY() + PathFollowerPosition.getInstance().getPathCentricY());

        Point2D.Double pA = new Point2D.Double(PathFollowerPosition.getInstance().getX(),PathFollowerPosition.getInstance().getY());
        Point2D.Double pB = new Point2D.Double(PathFollowerPosition.getInstance().getX() + Math.cos(Math.toRadians(robotAngle)) * 20,PathFollowerPosition.getInstance().getY() + Math.sin(Math.toRadians(robotAngle)) * 20);
        Point2D.Double pC = currentCircleCenterPoint;

        double sign = findSide(pC,pA,pB);


        Graph.getInstance().graphDebug(pC.getX(),pC.getY(), pA.getX(), pA.getY(), pB.getX(), pB.getY());


        this.currentRadius = Math.abs(Math.sqrt(circleCenter.getX() * circleCenter.getX() + circleCenter.getY() * circleCenter.getY())) * sign;
    }

    private TrajectoryPoint findClosestPoint() {
        double currentClosest = 9999;
        TrajectoryPoint closestPoint = null;
        for (int i = 0; i < path.getTrajectoryPoints().length; i++) {
            if (path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) < currentClosest) {
                currentClosest = path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY()));
                closestPoint = path.getTrajectoryPoints()[i];
            }
        }
        return closestPoint;
    }

    public void calculateTargetPoint() {
        this.currentClosestPoint = findClosestPoint();

        if (path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) < lookaheadDistance) {
            final double angle = path.getCoordinates()[path.getCoordinates().length - 1].getAngle();
            final double newLookahead = lookaheadDistance - path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY()));
            final double x = Math.cos(Math.toRadians(angle)) * newLookahead + path.getCoordinates()[path.getCoordinates().length - 1].getX();
            final double y = Math.sin(Math.toRadians(angle)) * newLookahead + path.getCoordinates()[path.getCoordinates().length - 1].getY();
            prevTargetIndex = path.getTrajectoryPoints().length - 1;
            currentTargetPoint = new TrajectoryPoint(x, y);
        } else {
            double currentClosest = 9999;
            for (int i = prevTargetIndex; i < path.getTrajectoryPoints().length; i++) {
                if (Math.abs(path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) - lookaheadDistance) < currentClosest) {
                    currentClosest = Math.abs(path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) - lookaheadDistance);
                    prevTargetIndex = i;
                    currentTargetPoint = path.getTrajectoryPoints()[i];
                }
            }
        }
    }

    private double findSide(Point2D.Double p, Point2D.Double p1, Point2D.Double p2){
        double x = p.getX();
        double y = p.getY();
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        return  -Math.signum((x-x1)*(y2-y1)-(y-y1)*(x2-x1));
    }

    public double getLookaheadDistance() {
        return lookaheadDistance;
    }

    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path path) {
        this.path = path;
    }

    public double getCurrentRadius() {
        return currentRadius;
    }

    public void setCurrentRadius(double currentRadius) {
        this.currentRadius = currentRadius;
    }

    public TrajectoryPoint getCurrentTargetPoint() {
        return currentTargetPoint;
    }

    public void setCurrentTargetPoint(TrajectoryPoint currentTargetPoint) {
        this.currentTargetPoint = currentTargetPoint;
    }
    public TrajectoryPoint getCurrentClosestPoint() {
        return currentClosestPoint;
    }

    public void setCurrentClosestPoint(TrajectoryPoint currentClosestPoint) {
        this.currentClosestPoint = currentClosestPoint;
    }

    public Point2D.Double getCurrentCircleCenterPoint() {
        return currentCircleCenterPoint;
    }

    public void setCurrentCircleCenterPoint(Point2D.Double currentCircleCenterPoint) {
        this.currentCircleCenterPoint = currentCircleCenterPoint;
    }
}
