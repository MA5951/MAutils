// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SingleMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class SingleMotor extends SubsystemBase implements MASubsystem {
  private MAMotorControler Motor;
  private static SingleMotor singleMotor;
  private MAShuffleboard singleMotorShuffleboard = new MAShuffleboard(SingleMotorConstants.KSUBSYSTEM_NAME);

  private SingleMotor() {
    Motor = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID1);
    setMAMotorComtrolersList(Motor);
  }

  @Override
  public void periodic() {
    PrintValues();
  }

  /**
   * Motor - 0
   */
  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).set(power);
  }

  @Override
  public void PrintValues() {
    singleMotorShuffleboard.addNum("MotorOutPut", maMotorControlers.get(0).getStatorCurrent());
    singleMotorShuffleboard.addNum("MotorOutPut", maMotorControlers.get(0).getOutput());
  }

  public static SingleMotor getinstance() {
    if (singleMotor == null) {
      singleMotor = new SingleMotor();
    }
    return singleMotor;
  }
}
