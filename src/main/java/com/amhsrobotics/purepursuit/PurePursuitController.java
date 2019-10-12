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
 *
 * References: http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
 * http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
 * https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 */
public class PurePursuitController {

    private double lookaheadDistance = 15;
    private double trackWidth = 27;
    private Path path;

    private double currentRadius;
    private TrajectoryPoint currentTargetPoint;

    public PurePursuitController(Path path){

        this.path = path;
    }

    public PurePursuitOutput update(){
        return new PurePursuitOutput(leftVelocityFromRadius(currentRadius),rightVelocityFromRadius(currentRadius));
    }

    public double leftVelocityFromRadius(double radius){
        double baseVelocity = 50;

        double angularVelocity = baseVelocity/radius;

        return angularVelocity*(radius - (trackWidth/2));
    }

    public double rightVelocityFromRadius(double radius){
        double baseVelocity = 50;

        double angularVelocity = baseVelocity/radius;

        return angularVelocity*(radius + (trackWidth/2));
    }

    public double calculateRadiusToTarget(double robotAngle){

        robotAngle = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,robotAngle), new CoordinateSystem(90, TurnSign.POSITIVE, VectorDirection.POSITIVE_X,VectorDirection.POSITIVE_Y)).getAngle();

        Point2D.Double vectorHead = new  Point2D.Double(Math.cos(Math.toRadians(robotAngle)),Math.sin(Math.toRadians(robotAngle)));

        TrajectoryPoint targetPoint = new TrajectoryPoint(40,40);

        double a = targetPoint.getX();
        double b = targetPoint.getY();
        double c = vectorHead.getX();
        double d = vectorHead.getY();

        double x = (d*(a*a+b*b))/(2*((a*d)-(b*c)));
        double y = -(c*(a*a+b*b))/(2*((a*d)-(b*c)));

        Point2D.Double circleCenter = new Point2D.Double(x,y);



        System.out.println("Center: " + circleCenter.getX() + " " + circleCenter.getY()+ " " + robotAngle + " " + vectorHead.getX() + " " + vectorHead.getY());

        this.currentRadius = Math.sqrt(circleCenter.getX()*circleCenter.getX() + circleCenter.getY()*circleCenter.getY());
        return this.currentRadius;
    }

    public void calculateRadiusToTarget(){

        double robotAngle = 0;

        robotAngle = CoordinateManager.getInstance().coordinateTransformation(new Coordinate(0,0,robotAngle), new CoordinateSystem(90, TurnSign.POSITIVE, VectorDirection.POSITIVE_X,VectorDirection.POSITIVE_Y)).getAngle();

        Point2D.Double vectorHead = new  Point2D.Double(Math.cos(Math.toRadians(robotAngle))*10,Math.sin(Math.toRadians(robotAngle))*10);

        TrajectoryPoint targetPoint = new TrajectoryPoint(40,40);

        double x = (vectorHead.getY() * (Math.pow(targetPoint.getX(),2)+Math.pow(targetPoint.getY(),2)))/(2*(targetPoint.getX()*vectorHead.getY()-targetPoint.getY()-vectorHead.getX()));
        double y = (vectorHead.getX() * (Math.pow(targetPoint.getX(),2)+Math.pow(targetPoint.getY(),2)))/(2*(targetPoint.getX()*vectorHead.getY()-targetPoint.getY()-vectorHead.getX()));

        Point2D.Double circleCenter = new Point2D.Double(x,y);



        this.currentRadius = Math.sqrt(circleCenter.getX()*circleCenter.getX() + circleCenter.getY()+circleCenter.getY());
    }

    public void calculateTargetPoint(){
       currentTargetPoint = new TrajectoryPoint(0,0);
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
