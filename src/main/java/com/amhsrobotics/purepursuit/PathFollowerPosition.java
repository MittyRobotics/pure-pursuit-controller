package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.coordinate.Coordinate;
import com.amhsrobotics.purepursuit.coordinate.CoordinateManager;
import com.amhsrobotics.purepursuit.coordinate.CoordinateSystem;
import com.amhsrobotics.purepursuit.coordinate.enums.TurnSign;
import com.amhsrobotics.purepursuit.coordinate.enums.VectorDirection;

public class PathFollowerPosition {
    private static PathFollowerPosition ourInstance = new PathFollowerPosition();

    public static PathFollowerPosition getInstance() {
        return ourInstance;
    }

    private PathFollowerPosition() {
    }

    private double x;
    private double y;
    private double heading;
    private double leftVelocity;
    private double rightVelocity;
    private double pathCentricX;
    private double pathCentricY;
    private double pathCentricHeading;
    private double trackWidth;

    /**
     * The coordinate system of the path calculator
     * <p>
     * Used to convert robot coordinates to path coordinates.
     */
    private CoordinateSystem PATH_COORDINATE_SYSTEM = new CoordinateSystem(
            90,
            TurnSign.POSITIVE,
            VectorDirection.POSITIVE_Y,
            VectorDirection.NEGATIVE_X);

    public void setupRobot(double trackWidth) {
        setTrackWidth(trackWidth);
    }

    /**
     * Updates the path follower's position with the robot's position
     * <p>
     * These coordinates should be in the standardized {@link CoordinateManager} world coordinate system.
     *
     * @param x       X position of the robot
     * @param y       Y position of the robot
     * @param heading Heading of the robot/
     */
    public void update(double x, double y, double heading, double leftVelocity, double rightVelocity) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        Coordinate pathCentricCoordinate = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(x, y, heading), PATH_COORDINATE_SYSTEM);
        this.pathCentricX = pathCentricCoordinate.getX();
        this.pathCentricY = pathCentricCoordinate.getY();
        this.pathCentricHeading = pathCentricCoordinate.getAngle();
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getPathCentricX() {
        return pathCentricX;
    }

    public double getPathCentricY() {
        return pathCentricY;
    }

    public double getPathCentricHeading() {
        return pathCentricHeading;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public void setLeftVelocity(double leftVelocity) {
        this.leftVelocity = leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public void setRightVelocity(double rightVelocity) {
        this.rightVelocity = rightVelocity;
    }
}
