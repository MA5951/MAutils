// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.utils.*;
import frc.robot.utils.Actuators.*;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;
import frc.robot.utils.MASubsystem.MASubsystem;

public class PistonIntake extends MASubsystem {

  private MAMotorControler intakeCollection;
  private MAPiston pistonA;
  private MAPiston pistonB;
  private static PistonIntake m_Intake;
  private MAShuffleboard intakesMaShuffleboard = new MAShuffleboard(IntakeConstants.KSUBSYSTEM_NAME);

  private PistonIntake() {
    intakeCollection = new MAMotorControler(MOTOR_CONTROLL.VICTOR, ID12);
    setMAMotorComtrolersList(intakeCollection);
    pistonA = new MAPiston(RobotConstants.P_ID2);
    pistonB = new MAPiston(RobotConstants.P_ID3);

  }

  @Override
  public void setPiston(boolean on) {
    pistonA.set(on);
    pistonB.set(on);

  }

  @Override
  public double getStatorCurrent() {
    return maMotorControlers.get(IntakeConstants.INTAKE_COLLECTION).getStatorCurrent();
  }

  @Override
  public void periodic() {
    printValues();
  }

  /**
   * index 0 - IntakeCollection
   */
  @Override
  public void setMotorPower(double power, int Index) {
    maMotorControlers.get(Index).set(power);
  }

  @Override
  public void printValues() {
    intakesMaShuffleboard.addNum("PistonIntakeGetStatorCurrent", getStatorCurrent());
    intakesMaShuffleboard.addBoolean("PistonIntakeOpenOrClose", pistonA.get());
  }

  public static PistonIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new PistonIntake();
    }
    return m_Intake;
  }
}
