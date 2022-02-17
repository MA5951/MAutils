package com.ma5951.utils.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MotorSubsystem extends Subsystem {
    public void setVoltage(double voltage);

    default void setPower(double power) {
        setVoltage(power * 12);
    }
}