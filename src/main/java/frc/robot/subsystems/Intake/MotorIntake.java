// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.utils.controlers.MAPidController;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MotorIntake extends MASubsystem {

  private MAMotorControler IntakeMove;
  private MAMotorControler IntakeCollection;
  private MAPidController IntakeMovePID;
  private static MotorIntake m_Intake;

  private MAShuffleboard motorIntakesShuffleboard = new MAShuffleboard(IntakeConstants.KSUBSYSTEM_NAME);

  private MotorIntake() {

    IntakeCollection = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID8);
    setMAMotorComtrolersList(IntakeCollection);

    // change the Limit
    IntakeMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID7, false, 0, false, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(IntakeMove);

    // cahnge Tolorance
    IntakeMovePID = new MAPidController(IntakeConstants.KP_INTAKE_MOVE, IntakeConstants.KI_INTAKE_MOVE,
        IntakeConstants.KD_INTAKE_MOVE, 0, 10, -1, 1);
    resetSensor();

  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * indax 0 - IntakeCollection indax 1 - IntakeMove
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
    return IntakeMovePID.calculate(getEncoderPosition(), setPoint);
  }

  @Override
  public boolean isPIDAtTarget(double waitTime) {
    return IntakeMovePID.atSetpoint(waitTime);
  }

  @Override
  public double getSetpointPID() {
    return IntakeMovePID.getSetpoint();
  }

  @Override
  public double getPositionError() {
    return IntakeMovePID.getPositionError();
  }

  @Override
  public double getStatorCurrent(int indax) {
    return maMotorControlers.get(indax).getStatorCurrent();
  }

  @Override
  public void PrintValues() {
    motorIntakesShuffleboard.addNum("MotorIntakeGetPosition", getEncoderPosition());
    motorIntakesShuffleboard.addNum("MotorIntakeGetSetPoint", getSetpointPID());
    motorIntakesShuffleboard.addNum("MotorIntakeGetPositionError", getPositionError());
    motorIntakesShuffleboard.addNum("MotorIntakeGetStatorCurrentMovetMotor", getStatorCurrent(IntakeConstants.INTAKE_MOVE));
    motorIntakesShuffleboard.addNum("MotorIntakeGetStatorCurrentCollection", getStatorCurrent(IntakeConstants.INTAKE_COLLECTION));
    motorIntakesShuffleboard.addBoolean("MotorIntakeAtSetPoint", isPIDAtTarget(0.1));
    motorIntakesShuffleboard.addBoolean("MotorIntakeLimitSwitchValuse", getLimitSwitchFValuse());
  }

  public static MotorIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new MotorIntake();
    }
    return m_Intake;
  }
}
