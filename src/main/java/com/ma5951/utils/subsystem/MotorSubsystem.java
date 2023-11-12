package com.ma5951.utils.subsystem;

public interface MotorSubsystem extends SimpleSubsystem {
    public void setVoltage(double voltage);

    default void setPower(double power) {
        setVoltage(power * 12);
    }
}