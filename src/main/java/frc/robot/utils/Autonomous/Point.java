// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Autonomous;

import frc.robot.subsystems.Chassis.ChassisConstants;

/** Add your docs here. */
public class Point {

    private double distance, angle, velocity = ChassisConstants.KMAX_SPEED,
            acceleration = ChassisConstants.KMAX_ACCELERATION;

    public Point(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }

    public Point(double distance, double angle, double velocity) {
        this.distance = distance;
        this.angle = angle;
        this.velocity = velocity;
    }

    public Point(double distance, double angle, double velocity, double acceleration) {
        this.distance = distance;
        this.angle = angle;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double getVelocity() {
        return velocity;
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

}
