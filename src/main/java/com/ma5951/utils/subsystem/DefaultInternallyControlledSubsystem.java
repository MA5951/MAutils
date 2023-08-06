package com.ma5951.utils.subsystem;

public interface DefaultInternallyControlledSubsystem extends
  InternallyControlledSubsystem{
  public void setSetPoint(double setPoint);

  public double getSetPoint();
}
