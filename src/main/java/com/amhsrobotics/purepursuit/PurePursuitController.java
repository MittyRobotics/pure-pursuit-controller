package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.paths.TrajectoryPoint;

import javax.swing.plaf.synth.SynthScrollBarUI;
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
    private double currentLookaheadDistance = 15;
    private double trackWidth = 20;
    private Path path;

    private double currentRadius;

    private double currentBaseVelocity = 0;

    private TrajectoryPoint currentClosestPoint;
    private TrajectoryPoint currentTargetPoint;
    private Point2D.Double currentCircleCenterPoint;
    private int prevTargetIndex = 0;
    private double timeSinceInitialized;
    private double currentPointAdjustedTime;

    
    private boolean isFinished;

    public PurePursuitController(Path path) {
        this.path = path;
    }

    public PurePursuitOutput update(double timeSinceInitialized) {
        this.timeSinceInitialized = timeSinceInitialized;
        calculateTargetPoint();
        calculateRadiusToTarget();
        
        double endThreshold = 1;
        isFinished = currentClosestPoint.distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) < endThreshold;
        
        return new PurePursuitOutput(leftVelocityFromRadius(), rightVelocityFromRadius());
    }

    public double leftVelocityFromRadius() {
        double baseVelocity = currentClosestPoint.getVelocity();

        this.currentBaseVelocity = baseVelocity;

        double angularVelocity = baseVelocity / getCurrentRadius();

        return angularVelocity * (getCurrentRadius() - (trackWidth / 2));
    }

    public double rightVelocityFromRadius() {
        double baseVelocity = currentClosestPoint.getVelocity();

        this.currentBaseVelocity = baseVelocity;

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

        this.currentRadius = Math.abs(Math.sqrt(circleCenter.getX() * circleCenter.getX() + circleCenter.getY() * circleCenter.getY())) * sign;
    }


    private TrajectoryPoint findClosestPoint() {
        double currentClosest = 9999;
        TrajectoryPoint point = null;
        for (int i = 0; i < path.getTrajectoryPoints().length; i++) {
            if (path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) < currentClosest) {
                currentClosest = path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY()));
                point = path.getTrajectoryPoints()[i];
            }
        }
        
        return point;
    }

    private TrajectoryPoint findClosestLookaheadPoint(){
        TrajectoryPoint point = null;
        if (path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) < currentLookaheadDistance) {
            final double angle = path.getCoordinates()[path.getCoordinates().length - 1].getAngle();
            final double newLookahead = currentLookaheadDistance - path.getTrajectoryPoints()[path.getTrajectoryPoints().length - 1].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY()));
            final double x = Math.cos(Math.toRadians(angle)) * newLookahead + path.getCoordinates()[path.getCoordinates().length - 1].getX();
            final double y = Math.sin(Math.toRadians(angle)) * newLookahead + path.getCoordinates()[path.getCoordinates().length - 1].getY();
            prevTargetIndex = path.getTrajectoryPoints().length - 1;
            point = new TrajectoryPoint(x, y);
        } else {
            double currentClosest = 9999;
            for (int i = prevTargetIndex; i < path.getTrajectoryPoints().length; i++) {
                if (Math.abs(path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) - currentLookaheadDistance) < currentClosest) {
                    currentClosest = Math.abs(path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) - currentLookaheadDistance);
                    prevTargetIndex = i;
                    point = path.getTrajectoryPoints()[i];
                }
            }
        }

        return point;
    }


    public void calculateTargetPoint() {
        this.currentClosestPoint = findClosestPoint();

        calculateAdaptiveLookaheadClosest();

        double adaptiveLookahead1 = currentLookaheadDistance;

        currentTargetPoint = findClosestLookaheadPoint();


        calculateAdaptiveLookaheadTarget();

        double adaptiveLookahead2 = currentLookaheadDistance;

        if(adaptiveLookahead2 < adaptiveLookahead1){
            currentTargetPoint = findClosestLookaheadPoint();
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

    private void calculateAdaptiveLookaheadTarget(){
        double maxLookahead = lookaheadDistance;
        double minLookahead = lookaheadDistance/5;

        double x = currentTargetPoint.getVelocity();
        double a = 0;
        double b = path.getVelocityConstraints().getMaxVelocity();
        double c = minLookahead;
        double d = maxLookahead;

        this.currentLookaheadDistance = (x-a)/(b-a) * (d-c) + c;
    }

    private void calculateAdaptiveLookaheadClosest(){
        double maxLookahead = lookaheadDistance;
        double minLookahead = lookaheadDistance/5;

        double x = currentClosestPoint.getVelocity();
        double a = 0;
        double b = path.getVelocityConstraints().getMaxVelocity();
        double c = minLookahead;
        double d = maxLookahead;

        this.currentLookaheadDistance = map(x,a,b,c,d);
    }

    public double map(double val, double valMin, double valMax, double desiredMin, double desiredMax){
        return (val-valMin)/(valMax-valMin) * (desiredMax-desiredMin)+desiredMin;
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

    public double getCurrentLookaheadDistance() {
        return currentLookaheadDistance;
    }

    public void setCurrentLookaheadDistance(double currentLookaheadDistance) {
        this.currentLookaheadDistance = currentLookaheadDistance;
    }

    public double getCurrentBaseVelocity() {
        return currentBaseVelocity;
    }

    public void setCurrentBaseVelocity(double currentBaseVelocity) {
        this.currentBaseVelocity = currentBaseVelocity;
    }

    public double getTimeSinceInitialized() {
        return timeSinceInitialized;
    }

    public void setTimeSinceInitialized(double timeSinceInitialized) {
        this.timeSinceInitialized = timeSinceInitialized;
    }

    public double getCurrentPointAdjustedTime() {
        return currentPointAdjustedTime;
    }

    public void setCurrentPointAdjustedTime(double currentPointAdjustedTime) {
        this.currentPointAdjustedTime = currentPointAdjustedTime;
    }
    
    
    public boolean isFinished() {
        return isFinished;
    }
    
    public void setIsFinished(boolean isFinished) {
        this.isFinished = isFinished;
    }
}
