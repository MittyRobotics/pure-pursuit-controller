package com.amhsrobotics.purepursuit;

import com.amhsrobotics.motorsim.graph.Graph;
import com.amhsrobotics.motorsim.math.Conversions;
import com.amhsrobotics.motorsim.motors.CIMMotor;
import com.amhsrobotics.motorsim.simulator.ControlLoop;
import com.amhsrobotics.motorsim.simulator.ControlLoopType;
import com.amhsrobotics.motorsim.simulator.ControlType;
import com.amhsrobotics.motorsim.simulator.MotorSimulator;

public class TestMain {
    public static void main(String[] args) {
        double mass = 40.0 * Conversions.LBS_TO_KG; //Kg
        double gearRatio = 7;
        double wheelRadius = 2 * Conversions.IN_TO_M; //Meters
        double maxVoltage = 12;

        double iterationTime = 0.02;

        ControlLoop controlLoop = new ControlLoop(ControlLoopType.VELOCITY, maxVoltage, iterationTime);
        controlLoop.setupVelocityController(0.073, 0, 0);
        ControlType controlType = ControlType.VELOCITY;

        MotorSimulator motorSimulator = new MotorSimulator(new CIMMotor(), 4, mass, gearRatio, wheelRadius, controlLoop, controlType, "CIM motor");
        Graph graph = new Graph("CIM Motor");

        double setpoint = 10 * Conversions.IN_TO_M;

        double t = 0.0;

        while (t < 20) {
            motorSimulator.update(setpoint, iterationTime);
            graph.addPosition(motorSimulator.getPosition(), t);
            graph.addVelocity(motorSimulator.getVelocity(), t);
            graph.addVoltage(motorSimulator.getVoltage(), t);
            graph.addSetpoint(setpoint, t);
            t += iterationTime;
        }
        System.out.println(calculateAngleToLookahead(0,10,10,5,5));
    }

    public static double calculateAngleToLookahead(double robotAngle, double lx, double ly, double rx, double ry){
        double x = lx - rx;
        double y = ly - ry;
        double angleToLookahead = robotAngle - Math.toDegrees(Math.atan2(y,x));
        return angleToLookahead;
    }
}
