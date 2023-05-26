package com.ma5951.utils.subsystem;

public interface ControlSubsystemInSubsystemControl extends MotorSubsystem{
  public void calculate(double setPoint);

  public boolean atPoint();
}
