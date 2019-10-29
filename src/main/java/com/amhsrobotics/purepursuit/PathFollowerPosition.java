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
    private double pathCentricX;
    private double pathCentricY;
    private double pathCentricHeading;
    private double trackWidth;

    private CoordinateSystem PATH_COORDINATE_SYSTEM = new CoordinateSystem(
            90,
            TurnSign.POSITIVE,
            VectorDirection.POSITIVE_Y,
            VectorDirection.NEGATIVE_X);

    public void setupRobot(double trackWidth){
        setTrackWidth(trackWidth);
    }
    
    
    /**
     * Updates the path follower's position with the robot's position
     * @param x         X position of the robot
     * @param y         Y position of the robot
     * @param heading   Heading of the robot/
     */
    public void update(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
        Coordinate pathCentricCoordinate = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(x,y,heading),PATH_COORDINATE_SYSTEM);
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

    public void setPathCentricX(double pathCentricX) {
        this.pathCentricX = pathCentricX;
    }

    public double getPathCentricY() {
        return pathCentricY;
    }

    public void setPathCentricY(double pathCentricY) {
        this.pathCentricY = pathCentricY;
    }

    public double getPathCentricHeading() {
        return pathCentricHeading;
    }

    public void setPathCentricHeading(double pathCentricHeading) {
        this.pathCentricHeading = pathCentricHeading;
    }
    
    public double getTrackWidth() {
        return trackWidth;
    }
    
    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }
}
