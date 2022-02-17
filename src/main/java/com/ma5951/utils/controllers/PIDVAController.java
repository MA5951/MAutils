// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import com.ma5951.utils.DriverStation;
import com.ma5951.utils.controllers.interfaces.BasePIDVAController;

public class PIDVAController implements BasePIDVAController {
    private ProfiledPIDController profiledPIDController;
    private SimpleMotorFeedforward feed;
    private double low = -12, high = 12;
    private double acceleration = 0;

    public PIDVAController(double kp, double ki, double kd, double tolerance, double maxVelocity, double maxAcceleration) {
        profiledPIDController = new ProfiledPIDController(kp, ki, kd,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        profiledPIDController.setTolerance(tolerance, tolerance);
        setFeedforward(0, 0, 0);
        acceleration = maxAcceleration;
    }

    public PIDVAController(double kp, double ki, double kd, double tolerance, double maxVelocity, double maxAcceleration,
                           double ks, double kv, double ka) {
        profiledPIDController = new ProfiledPIDController(kp, ki, kd,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        profiledPIDController.setTolerance(tolerance, tolerance);
        setFeedforward(ks, kv, ka);
        acceleration = maxAcceleration;
    }

    private void setFeedforward(double ks, double kv, double ka) {
        feed = new SimpleMotorFeedforward(ks, kv, ka);
    }

    @Override
    public void setOutputRange(double low, double high) {
        this.low = low;
        this.high = high;

    }

    @Override
    public void setPID(double kp, double ki, double kd) {
        profiledPIDController.setPID(kp, ki, kd);

    }

    @Override
    public void setI(double ki) {
        profiledPIDController.setI(ki);

    }

    @Override
    public void setP(double kp) {
        profiledPIDController.setP(kp);
    }

    @Override
    public void setD(double kd) {
        profiledPIDController.setD(kd);
    }

    @Override
    public void setF(double kf) {
        DriverStation.printError("setF in MAPIDVA dont work", true);
    }

    @Override
    public double getP() {
        return profiledPIDController.getP();
    }

    @Override
    public double getI() {
        return profiledPIDController.getI();
    }

    @Override
    public double getD() {
        return profiledPIDController.getD();
    }

    @Override
    public double getF() {
        return feed.calculate(profiledPIDController.getSetpoint().velocity, acceleration);
    }

    @Override
    public void enableContinuousInput(double minimumInput, double maximumInput) {
        profiledPIDController.enableContinuousInput(minimumInput, maximumInput);

    }

    @Override
    public void disableContinuousInput() {
        profiledPIDController.disableContinuousInput();

    }

    @Override
    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        profiledPIDController.setIntegratorRange(minimumIntegral, maximumIntegral);

    }

    @Override
    public void setSetpoint(double setpoint) {
        profiledPIDController.setGoal(setpoint);

    }

    @Override
    public boolean atSetpoint() {
        return profiledPIDController.atGoal();
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    @Override
    public double calculate(double measurement) {
        double value = profiledPIDController.calculate(measurement)
                + feed.calculate(profiledPIDController.getSetpoint().velocity, acceleration);
        return MathUtil.clamp(value, low, high);
    }

    @Override
    public double getPositionError() {
        return profiledPIDController.getPositionError();
    }

    @Override
    public double getSetpoint() {
        return profiledPIDController.getGoal().position;
    }

    @Override
    public void reset() {
        profiledPIDController.reset(profiledPIDController.getSetpoint().position); // TODO

    }

    @Override
    public void setConstraints(double maxVelocity, double maxAcceleration) {
        acceleration = maxAcceleration;
        profiledPIDController.setConstraints(new Constraints(maxVelocity, maxAcceleration));

    }

    @Override
    public void setGoal(double position, double velocity) {
        profiledPIDController.setGoal(new State(position, velocity));

    }

    @Override
    public double getVelocitySetpoint() {
        return profiledPIDController.getGoal().velocity;
    }

    @Override
    public void reset(double position, double velocity) {
        profiledPIDController.reset(position, velocity);

    }

    @Override
    public double calculate(double measurement, double positionSetpoint, double velocitySetpoint) {
        setGoal(positionSetpoint, velocitySetpoint);
        return calculate(measurement);
    }

    @Override
    public double calculate(double measurement, double positionSetpoint, double velocitySetpoint, double maxVelocity,
                            double maxAcceleration) {
        acceleration = maxAcceleration;
        setConstraints(maxVelocity, maxAcceleration);
        return calculate(measurement, positionSetpoint, velocitySetpoint);
    }

}
