// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.Controlers.Interfaces.Controler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MotorIntake extends MASubsystem {

  private MAMotorControler intakeMove;
  private MAMotorControler intakeCollection;
  private Controler intakeMovePID;
  private static MotorIntake m_Intake;

  private MAShuffleboard intakesShuffleboard = new MAShuffleboard(IntakeConstants.KSUBSYSTEM_NAME);

  private MotorIntake() {

    intakeCollection = new MAMotorControler(MOTOR_CONTROLL.TALON, ID8);
    setMAMotorComtrolersList(intakeCollection);

    // change the Limit
    intakeMove = new MAMotorControler(MOTOR_CONTROLL.TALON, ID7, false, 0, false, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(intakeMove);

    // cahnge Tolerance
    intakeMovePID = new MAPidController(IntakeConstants.KP_INTAKE_MOVE, IntakeConstants.KI_INTAKE_MOVE,
        IntakeConstants.KD_INTAKE_MOVE, 0, 10, -1, 1);
    resetSensor();

  }

  @Override
  public void periodic() {
    printValues();

  }

  /**
   * index 0 - IntakeCollection index 1 - IntakeMove
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).set(power);
  }

  @Override
  public double getEncoderPosition() {
    return maMotorControlers.get(IntakeConstants.INTAKE_MOVE).getPosition();
  }

  @Override
  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(IntakeConstants.INTAKE_MOVE).getForwardLimitSwitch();
  }

  @Override
  public void resetSensor() {
    maMotorControlers.get(IntakeConstants.INTAKE_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(IntakeConstants.INTAKE_MOVE).overrideLimitSwitches(overrid);
  }

  @Override
  public double calculatePIDOutput(double setPoint) {
    return intakeMovePID.calculate(getEncoderPosition(), setPoint);
  }

  @Override
  public boolean isPIDAtTarget() {
    return intakeMovePID.atSetpoint();
  }

  @Override
  public double getSetpointPID() {
    return intakeMovePID.getSetpoint();
  }

  @Override
  public double getPositionError() {
    return intakeMovePID.getPositionError();
  }

  @Override
  public double getStatorCurrent(int index) {
    return maMotorControlers.get(index).getStatorCurrent();
  }

  @Override
  public void printValues() {
    intakesShuffleboard.addNum("MotorIntakeGetPosition", getEncoderPosition());
    intakesShuffleboard.addNum("MotorIntakeGetSetPoint", getSetpointPID());
    intakesShuffleboard.addNum("MotorIntakeGetPositionError", getPositionError());
    intakesShuffleboard.addNum("MotorIntakeGetStatorCurrentMovetMotor", getStatorCurrent(IntakeConstants.INTAKE_MOVE));
    intakesShuffleboard.addNum("MotorIntakeGetStatorCurrentCollection",
        getStatorCurrent(IntakeConstants.INTAKE_COLLECTION));
    intakesShuffleboard.addBoolean("MotorIntakeAtSetPoint", isPIDAtTarget());
    intakesShuffleboard.addBoolean("MotorIntakeLimitSwitchValuse", getLimitSwitchFValuse());
  }

  public static MotorIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new MotorIntake();
    }
    return m_Intake;
  }
}
