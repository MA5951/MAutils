package com.ma5951.utils.subsystem.chassis;

public interface BaseChassisPIDSubsystem extends BaseChassisSubsystem {
    public boolean isPIDRightVelocityAtSetpoint();

    public boolean isPIDLeftVelocityAtSetpoint();

    public void setRightVelocitySetpoint(double setpoint);

    public void setLeftVelocitySetpoint(double setpoint);

    public double getRightPID();

    public double getLeftPID();

}
