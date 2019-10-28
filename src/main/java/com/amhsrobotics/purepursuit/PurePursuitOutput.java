package com.amhsrobotics.purepursuit;

public class PurePursuitOutput {

    private double leftVelocity;
    private double rightVelocity;

    public PurePursuitOutput(double leftVelocity, double rightVelocity){

        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
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
