// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Autonomous;

import frc.robot.subsystems.Chassis.ChassisConstants;
import frc.robot.utils.Autonomous.Autonomous.autonomousState;

/** Add your docs here. */
public class Point {

    private double distance, angle, maxVelocity = ChassisConstants.KMAX_SPEED,
            acceleration = ChassisConstants.KMAX_ACCELERATION, endVelocity = ChassisConstants.KMAX_SPEED;

    public Point(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }

    public Point(double distance, double angle, double maxVelocity, double endVelocity) {
        this.distance = distance;
        this.angle = angle;
        this.endVelocity = endVelocity;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration * Math.abs(maxVelocity) / maxVelocity;

    }

    public Point(double distance, double angle, double maxVelocity, double endVelocity, double acceleration) {
        this.distance = distance;
        this.angle = angle;
        this.maxVelocity = maxVelocity;
        this.endVelocity = endVelocity;
        this.acceleration = acceleration;

    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getArcDistance() {
        if (Autonomous.state == autonomousState.RIGHT) {
            return Autonomous.getLeftCircelRadius() * Autonomous.getDeltaTheta();
        } else if (Autonomous.state == autonomousState.LEFT) {
            return Autonomous.getRightCircelRadius() * Autonomous.getDeltaTheta();
        } else if (Autonomous.state == autonomousState.TURN_IN_PLACE) {
            return Autonomous.getLeftCircelRadius() * Autonomous.getDeltaTheta();
        } else {
            return 0;
        }
    }

}
