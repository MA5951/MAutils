// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.controllers.interfaces;

/** Add your docs here. */
public interface PIDVAControler extends PIDControler {
    public void setConstraints(double maxVelocity, double maxAcceleration);

    public void setGoal(double position, double velocity);

    public double getVelocitySetpoint();

    public void reset(double position, double velocity);

    public double calculate(double measurement, double positionSetpoint, double velocitySetpoint);

    public double calculate(double measurement, double positionSetpoint, double velocitySetpoint, double maxVelocity,
            double maxAcceleration);
}
