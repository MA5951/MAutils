package com.ma5951.utils.subsystem;

public interface InternallyControlledSubsystem extends MotorSubsystem{
  public void calculate(double setPoint);

  public boolean atPoint();
}
