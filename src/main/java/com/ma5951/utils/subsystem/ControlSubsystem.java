package com.ma5951.utils.subsystem;

public interface ControlSubsystem extends MotorSubsystem {
    public void setSetpoint(double setpoint);

    public double getMeasurement();
}