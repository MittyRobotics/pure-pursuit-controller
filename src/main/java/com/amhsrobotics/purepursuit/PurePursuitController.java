package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;
import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.paths.TrajectoryPoint;

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
    private double trackWidth = 27;
    private Path path;

    private double currentRadius;
    private TrajectoryPoint currentClosestPoint;
    private TrajectoryPoint currentTargetPoint;
    private int prevTargetIndex;

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
        final double robotAngle = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0, 0, PathFollowerPosition.getInstance().getHeading()), new CoordinateSystem(90, TurnSign.POSITIVE, VectorDirection.POSITIVE_X, VectorDirection.POSITIVE_Y)).getAngle();

        final Point2D.Double vectorHead = new Point2D.Double(Math.cos(Math.toRadians(robotAngle)), Math.sin(Math.toRadians(robotAngle)));

        final double a = this.currentTargetPoint.getX();
        final double b = this.currentTargetPoint.getY();
        final double c = vectorHead.getX();
        final double d = vectorHead.getY();

        final double v = 2 * ((a * d) - (b * c));
        final double x = (d * ((a * a) + (b * b))) / v;
        final double y = -(c * (a * a + b * b)) / v;

        final Point2D.Double circleCenter = new Point2D.Double(x, y);

        this.currentRadius = Math.sqrt(circleCenter.getX() * circleCenter.getX() + circleCenter.getY() * circleCenter.getY());
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
            final double x = Math.cos(Math.toRadians(angle)) * lookaheadDistance;
            final double y = Math.sin(Math.toRadians(angle)) * lookaheadDistance;
            prevTargetIndex = path.getTrajectoryPoints().length - 1;
            currentTargetPoint = new TrajectoryPoint(x, y);
        } else {

            int currentClosest = 9999;
            for (int i = prevTargetIndex; i < path.getTrajectoryPoints().length; i++) {
                if (path.getTrajectoryPoints()[i].distance(new TrajectoryPoint(PathFollowerPosition.getInstance().getX(), PathFollowerPosition.getInstance().getY())) - lookaheadDistance < currentClosest) {
                    prevTargetIndex = i;
                    currentTargetPoint = path.getTrajectoryPoints()[i];
                }
            }
        }
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
}
