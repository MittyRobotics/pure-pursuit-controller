package com.amhsrobotics.purepursuit;

import com.amhsrobotics.purepursuit.paths.Path;
import com.amhsrobotics.purepursuit.paths.TrajectoryPoint;

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

    public void calculateRadiusToTarget(){
        this.currentRadius = 0;
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
