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

  private MAMotorControler IntakeCollection;
  private MAPiston PistonA;
  private MAPiston PistonB;
  private static PistonIntake m_Intake;
  private MAShuffleboard PistonIntakesMaShuffleboard = new MAShuffleboard(IntakeConstants.KSUBSYSTEM_NAME); // TODO

  private PistonIntake() {
    IntakeCollection = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID9);
    setMAMotorComtrolersList(IntakeCollection);
    PistonA = new MAPiston(RobotConstants.P_ID0);
    PistonB = new MAPiston(RobotConstants.P_ID1);
 

  }

  @Override
  public void setPiston(boolean on) {
    PistonA.set(on);
    PistonB.set(on);

  }

  @Override
  public double getStatorCurrent() {
    return maMotorControlers.get(IntakeConstants.INTAKE_COLLECTION).getStatorCurrent();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * indax 0 - IntakeCollection
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).set(power);
  }

  @Override
  public void PrintValues() {
    PistonIntakesMaShuffleboard.addNum("PistonIntakeGetStatorCurrent", getStatorCurrent());
    PistonIntakesMaShuffleboard.addBoolean("PistonIntakeOpenOrClose", PistonA.get());
  }

  public static PistonIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new PistonIntake();
    }
    return m_Intake;
  }
}
