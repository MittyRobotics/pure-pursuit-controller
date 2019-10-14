package com.amhsrobotics.purepursuit;

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
}
