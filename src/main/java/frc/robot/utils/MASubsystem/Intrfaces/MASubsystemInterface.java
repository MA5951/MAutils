/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.MASubsystem.Intrfaces;

public interface MASubsystemInterface {

  public void setMotorPower(double power, int index);

  public void setMotorVoltage(double voltage, int index);

  public double getEncoderPosition(int index);

  public double getEncdoerRPM(int index);

  public double getEncoderPosition();

  public double getEncdoerRPM();

  public void setPiston(boolean on);

  public void togglePiston();

  public boolean getLimitSwitchFValuse();

  public boolean getLimitSwitchRValuse();

  public boolean getPistonValue();

  public boolean getLimitSwitchFValuse(int index);

  public boolean getLimitSwitchRValuse(int index);

  public void resetSensor();

  public double calculatePIDOutput(double setPoint);

  public double calculatePIDOutput();

  public void setSetPoint(double setPoint);

  public double getStatorCurrent();

  public double resetPID();

  public double getStatorCurrent(int index);

  public boolean isPIDAtTarget();

  public double getSetpointPID();

  public double getPositionError();

  public void printValues();

  public double getOutput();
}
