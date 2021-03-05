// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Balance;

import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Balance extends MASubsystem {
  private MAMotorControler BalanceMotor;
  private static Balance m_Balance;
  private MAShuffleboard balanceShuffleboard = new MAShuffleboard(BalanceConstants.KSUBSYSTEM_NAME);

  private Balance() {
    BalanceMotor = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID9);
    setMAMotorComtrolersList(BalanceMotor);

  }

  @Override
  public void periodic() {
    printValues();
  }

  /**
   * Motor - 0
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).set(power);
  }

  @Override
  public double getStatorCurrent() {
    return maMotorControlers.get(BalanceConstants.MOTOR).getStatorCurrent();
  }

  @Override
  public double getOutput() {
    return maMotorControlers.get(BalanceConstants.MOTOR).getOutput();
  }

  @Override
  public void printValues() {
    balanceShuffleboard.addNum("SingleMotorgetStatorCurrent", maMotorControlers.get(BalanceConstants.MOTOR).getStatorCurrent());
    balanceShuffleboard.addNum("SingalMotorgetOutput", maMotorControlers.get(BalanceConstants.MOTOR).getOutput());
  }

  public static Balance getinstance() {
    if (m_Balance == null) {
      m_Balance = new Balance();
    }
    return m_Balance;
  }
}