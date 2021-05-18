// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SingleMotor;

import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class SingleMotor extends MASubsystem {
  private MAMotorControler motor;
  private static SingleMotor singleMotor;
  private MAShuffleboard singleMotorShuffleboard = new MAShuffleboard(SingleMotorConstants.KSUBSYSTEM_NAME);

  private SingleMotor() {
    motor = new MAMotorControler(MOTOR_CONTROLL.TALON, ID15);
    setMAMotorComtrolersList(motor);

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
    return maMotorControlers.get(0).getStatorCurrent();
  }

  @Override
  public double getOutput() {
    return maMotorControlers.get(0).getOutput();
  }

  @Override
  public void printValues() {
    singleMotorShuffleboard.addNum("SingleMotorgetStatorCurrent", maMotorControlers.get(0).getStatorCurrent());
    singleMotorShuffleboard.addNum("SingalMotorgetOutput", maMotorControlers.get(0).getOutput());
  }

  public static SingleMotor getinstance() {
    if (singleMotor == null) {
      singleMotor = new SingleMotor();
    }
    return singleMotor;
  }
}