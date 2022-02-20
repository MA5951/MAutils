package com.ma5951.utils.motor;

public interface MotorSensors {

    public void resetEncoder();

    public double getPosition();

    public double getVelocity();

    public double getStatorCurrent();

    public void phaseSensor(boolean phaseSensor);

    public void setCurrentLimit(int limit);

    public boolean getForwardLimitSwitch();

    public boolean getReverseLimitSwitch();
}
