// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPiston;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class PistonIntake extends SubsystemBase implements MASubsystem {

  private MAMotorControler IntakeCollection;
  private MAPiston pistonA;
  private MAPiston pistonB;
  private static PistonIntake m_Intake;
  private static final int INTAKE_COLLECTION = 0;
  private MAShuffleboard pistonIntakesMaShuffleboard = new MAShuffleboard(""); // TODO

  private PistonIntake() {
    IntakeCollection = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID1);
    setMAMotorComtrolers(IntakeCollection);

    pistonA = new MAPiston(RobotConstants.p_ID0);
    pistonB = new MAPiston(RobotConstants.p_ID1);

  }

  public void setPiston(boolean on) {
    pistonA.set(on);
    pistonB.set(on);
  }

  public double getStatorCurrent() {
    return maMotorControlers.get(INTAKE_COLLECTION).getStatorCurrent();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * indax 0 - IntakeCollection
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(INTAKE_COLLECTION).set(Power);
  }

  @Override
  public void PrintValues() {
    pistonIntakesMaShuffleboard.addNum("getStatorCurrent", getStatorCurrent());
    pistonIntakesMaShuffleboard.addBoolean("OpenOrClose", pistonA.get());
  }

  public static PistonIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new PistonIntake();
    }
    return m_Intake;
  }
}
