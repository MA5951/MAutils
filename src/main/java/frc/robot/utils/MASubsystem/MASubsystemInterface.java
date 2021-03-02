/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.MASubsystem;

public interface MASubsystemInterface {

  public void setMotorPower(double power, int indax);

  public void setMotorVoltage(double voltage, int indax);

  public double getEncoderPosition(int indax);

  public double getEncdoerRPM(int indax);

  public double getEncoderPosition();

  public double getEncdoerRPM();

  public void setPiston(boolean on);

  public void togglePiston();

  public boolean getLimitSwitchFValuse();

  public boolean getLimitSwitchRValuse();

  public boolean getLimitSwitchFValuse(int indax);

  public boolean getLimitSwitchRValuse(int indax);

  public void resetSensor();

  public double calculatePIDOutput(double setPoint);

  public double calculatePIDOutput();

  public void setSetPoint(double setPoint);

  public double getStatorCurrent();

  public double resetPID();

  public double getStatorCurrent(int indax);

  public boolean isPIDAtTarget(double waitTime);

  public double getSetpointPID();

  public double getPositionError();

  public void PrintValues();

  public double getOutput();
}
