package com.ma5951.utils.subsystem;

public interface ControlSubsystem extends MotorSubsystem {
    public void setSetpoint(double setpoint);

    public boolean atSetpoint();

    default double calculate(double setpoint) {
        setSetpoint(setpoint);
        return calculate();
    }

    public double calculate();
}