package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.paths.TrajectoryPoint;

import java.awt.geom.Point2D;

/**
 * Pure Pursuit Controller class
 * <p>
 * References:
 * Differential drive kinematics:
 * http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 * http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
 * Pure pursuit controller algorithm:
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
 */
public class PurePursuitController {

    private double lookaheadDistance;
    private double minLookaheadDistance;
    private double currentLookaheadDistance;
    private Path path;
    private double currentRadius;
    private double currentBaseVelocity;
    private TrajectoryPoint currentClosestPoint;
    private TrajectoryPoint currentTargetPoint;
    private Point2D.Double currentCircleCenterPoint;
    private double adaptiveDistanceGain;
    private double adjustedCurrentVelocity;
    private int prevTargetIndex;
    private double kCurvature;
    private double time;
    private double prevTime;
    private boolean reversed;
    private boolean isFinished;

    /**
     * Constructor
     *
     * @param path the {@link Path} object to follow
     */
    public PurePursuitController(Path path, boolean reversed) {
        this(path, 15, 10, reversed);
    }

    /**
     * Constructor
     *
     * @param path the {@link Path} object to follow
     */
    public PurePursuitController(Path path, double defaultLookaheadDistance, double minLookaheadDistance, boolean reversed) {
        this.path = path;
        this.lookaheadDistance = defaultLookaheadDistance;
        this.minLookaheadDistance = minLookaheadDistance;
        this.reversed = reversed;
    }


    /**
     * Updates the pure pursuit controller and returns a {@link PurePursuitOutput} object containing the left and right wheel velocities
     *
     * @return a {@link PurePursuitOutput} object containing the left and right wheel velocities;
     */
    public PurePursuitOutput update(double time) {
        this.time = time;

        calculateTargetPoint();
        calculateRadiusToTarget();

        double endThreshold = 1;
        isFinished = currentClosestPoint.distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) < endThreshold;

        calculateAdaptiveVelocity();


        double left = leftVelocityFromRadius();
        double right = rightVelocityFromRadius();

        if (reversed) {
            left = -left;
            right = -right;
        }

        this.prevTime = time;

        return new PurePursuitOutput(left, right,calculateAngleToLookahead());
    }

    /**
     * Returns the left wheel velocity
     * <p>
     * Applies differential drive kinematics to get the left wheel velocity from the current radius towards the lookahead point.
     *
     * @return the left wheel velocity
     */
    private double leftVelocityFromRadius() {
        double baseVelocity = Math.min(currentClosestPoint.getVelocity(), adjustedCurrentVelocity);

        this.currentBaseVelocity = baseVelocity;

        double angularVelocity = baseVelocity / getCurrentRadius();

        if (PathFollowerPosition.getInstance().getTrackWidth() != 0) {
            return limitLeftVelocityToConstraints(angularVelocity * (getCurrentRadius() - (PathFollowerPosition.getInstance().getTrackWidth() / 2)));
        } else {
            System.out.println("Pure pursuit controller trackWidth not setup!");
            return 0;
        }
    }

    /**
     * Returns the right wheel velocity
     * <p>
     * Applies differential drive kinematics to get the right wheel velocity from the current radius towards the lookahead point.
     *
     * @return the right wheel velocity
     */
    private double rightVelocityFromRadius() {
        double baseVelocity = Math.min(currentClosestPoint.getVelocity(), adjustedCurrentVelocity);

        this.currentBaseVelocity = baseVelocity;

        double angularVelocity = baseVelocity / getCurrentRadius();

        if (PathFollowerPosition.getInstance().getTrackWidth() != 0) {
            return limitRightVelocityToConstraints(angularVelocity * (getCurrentRadius() + (PathFollowerPosition.getInstance().getTrackWidth() / 2)));
        } else {
            System.out.println("Pure pursuit controller trackWidth not setup!");
            return 0;
        }
    }

    /**
     * Limits the output velocity for the left wheel to the velocity constraints.
     *
     * @param desiredVelocity the desired left wheel velocity
     * @return the limited left wheel velocity
     */
    private double limitLeftVelocityToConstraints(double desiredVelocity) {
        return desiredVelocity;
    }

    /**
     * Limits the output velocity for the right wheel to the velocity constraints.
     *
     * @param desiredVelocity the desired right wheel velocity
     * @return the limited right wheel velocity
     */
    private double limitRightVelocityToConstraints(double desiredVelocity) {
        return desiredVelocity;
    }
    
    /**
     * Calculates the radius for the robot to follow to the current target point
     */
    private void calculateRadiusToTarget() {

        final double robotAngle = PathFollowerPosition.getInstance().getPathCentricHeading();

        final Point2D.Double vectorHead = new Point2D.Double(Math.cos(Math.toRadians(robotAngle)), Math.sin(Math.toRadians(robotAngle)));

        final double a = this.currentTargetPoint.getX() - PathFollowerPosition.getInstance().getX();
        final double b = this.currentTargetPoint.getY() - PathFollowerPosition.getInstance().getY();
        final double c = vectorHead.getX();
        final double d = vectorHead.getY();

        final double v = 2 * ((a * d) - (b * c));
        final double x = (d * ((a * a) + (b * b))) / v;
        final double y = -(c * (a * a + b * b)) / v;


        final Point2D.Double circleCenter = new Point2D.Double(x, y);

        currentCircleCenterPoint = new Point2D.Double(circleCenter.getX() + PathFollowerPosition.getInstance().getPathCentricX(), circleCenter.getY() + PathFollowerPosition.getInstance().getPathCentricY());

        Point2D.Double pA = new Point2D.Double(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY());
        Point2D.Double pB = new Point2D.Double(PathFollowerPosition.getInstance().getX() + Math.cos(Math.toRadians(robotAngle)) * 20, PathFollowerPosition.getInstance().getY() + Math.sin(Math.toRadians(robotAngle)) * 20);
        Point2D.Double pC = currentCircleCenterPoint;

        double sign = findSide(pC, pA, pB);

        this.currentRadius = Math.abs(Math.sqrt(circleCenter.getX() * circleCenter.getX() + circleCenter.getY() * circleCenter.getY())) * sign;
    }

    /**
     * Finds the closest point to the robot.
     *
     * @return the closest point to the robot
     */
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

    /**
     * Finds the closest point that is currentLookaheadDistance away from the robot. This becomes the targetPoint
     *
     * @return
     */
    private TrajectoryPoint findClosestLookaheadPoint() {
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

    /**
     * Calculates the target point
     */
    private void calculateTargetPoint() {
        this.currentClosestPoint = findClosestPoint();

        calculateAdaptiveLookahead();

        currentTargetPoint = findClosestLookaheadPoint();
    }

    /**
     * Finds which side of a line the point is on
     *
     * @param p  point
     * @param p1 point 1 of the line
     * @param p2 point 2 of the line
     * @return a -1 or 1 value based on which side the point is on
     */
    private double findSide(Point2D.Double p, Point2D.Double p1, Point2D.Double p2) {
        double x = p.getX();
        double y = p.getY();
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        return -Math.signum((x - x1) * (y2 - y1) - (y - y1) * (x2 - x1));
    }

    /**
     * Calculates the adaptive lookahead distance
     * <p>
     * The adaptive lookahead distance is based on the velocity of the current point.
     */
    private void calculateAdaptiveLookahead() {
//Adaptive Lookahead from velocity of closest point
//        double x = currentClosestPoint.getVelocity();
//        double a = path.getVelocityConstraints().getMinVelocity();
//        double b = path.getVelocityConstraints().getMaxVelocity();
//        double c = minLookaheadDistance;
//        double d = lookaheadDistance;
//
//        this.currentLookaheadDistance = Math.max(map(x, a, b, c, d), minLookaheadDistance);

        //Adaptive Lookahead from distance away from path
        double distanceToPath = currentClosestPoint.distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY()));
        distanceToPath = distanceToPath * adaptiveDistanceGain;
        this.currentLookaheadDistance = lookaheadDistance + distanceToPath;
    }

    private void calculateAdaptiveVelocity(){
        this.adjustedCurrentVelocity = Math.abs(kCurvature / (1/currentRadius));
    }

    /**
     * Maps a value between valMin and valMax to desiredMin and desiredMax
     *
     * @param val        the value to map
     * @param valMin     the minimum range of the value
     * @param valMax     the maximum range of the value
     * @param desiredMin the mapped minimum range of the value
     * @param desiredMax the mapped maximum range of the value
     * @return the value between valMin and valMax mapped between desiredMin and desiredMax
     */
    private double map(double val, double valMin, double valMax, double desiredMin, double desiredMax) {
        return (val - valMin) / (valMax - valMin) * (desiredMax - desiredMin) + desiredMin;
    }

    /**
     * Returns the angle from the robot's current angle to the lookahead point
     *
     * @return the angle from the robot's current angle to the lookahead point
     */
    private double calculateAngleToLookahead(){
        double robotAngle = PathFollowerPosition.getInstance().getPathCentricHeading();
        double rx = PathFollowerPosition.getInstance().getPathCentricX();
        double ry = PathFollowerPosition.getInstance().getPathCentricY();
        double lx = currentTargetPoint.getX();
        double ly = currentTargetPoint.getY();
        double x = lx - rx;
        double y = ly - ry;
        double angleToLookahead = robotAngle - Math.toDegrees(Math.atan2(y,x));
        return angleToLookahead;
    }

    public double getLookaheadDistance() {
        return lookaheadDistance;
    }

    public void setLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
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

    public boolean isFinished() {
        return isFinished;
    }

    public void setIsFinished(boolean isFinished) {
        this.isFinished = isFinished;
    }

    public double getTime() {
        return time;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public double getPrevTime() {
        return prevTime;
    }

    public void setPrevTime(double prevTime) {
        this.prevTime = prevTime;
    }

    public boolean isReversed() {
        return reversed;
    }

    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }

    public double getAdaptiveDistanceGain() {
        return adaptiveDistanceGain;
    }

    public void setAdaptiveDistanceGain(double adaptiveDistanceGain) {
        this.adaptiveDistanceGain = adaptiveDistanceGain;
    }

    public double getAdjustedCurrentVelocity() {
        return adjustedCurrentVelocity;
    }

    public void setAdjustedCurrentVelocity(double adjustedCurrentVelocity) {
        this.adjustedCurrentVelocity = adjustedCurrentVelocity;
    }
    public double getkCurvature() {
        return kCurvature;
    }

    public void setkCurvature(double kCurvature) {
        this.kCurvature = kCurvature;
    }

}
