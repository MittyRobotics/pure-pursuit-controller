package com.amhsrobotics.purepursuit;

public class PurePursuitOutput {

    private double leftVelocity;
    private double rightVelocity;
    private double angleToLookahead;

    public PurePursuitOutput(double leftVelocity, double rightVelocity, double angleToLookahead){
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.angleToLookahead = angleToLookahead;
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

    public double getAngleToLookahead() {
        return angleToLookahead;
    }

    public void setAngleToLookahead(double angleToLookahead) {
        this.angleToLookahead = angleToLookahead;
    }
}
