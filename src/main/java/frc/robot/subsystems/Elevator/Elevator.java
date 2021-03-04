// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Elevator extends MASubsystem {

  private MAMotorControler ElevatorMove;
  private MAPidController ElevatorMovePID;
  private static Elevator m_Elevator;
  private MAShuffleboard ElevatoShuffleboard = new MAShuffleboard(ElevatorConstants.KSUBSYSTEM_NAME);

  private Elevator() {
    ElevatorMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID6, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(ElevatorMove);

    // cahnge Tolorance
    ElevatorMovePID = new MAPidController(ElevatorConstants.KP_ELEVATOR_MOVE, ElevatorConstants.KI_ELEVATOR_MOVE,
        ElevatorConstants.KD_ELEVATOR_MOVE, ElevatorConstants.KF_ELEVATOR_MOVE, 10, -12, 12);
    resetSensor();

  }

  @Override
  public void periodic() {
    PrintValues();
  }

  /**
   * set voltage -12 to 12 indax 0 - ElevatorMove
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).setVoltage(power);
  }

  @Override
  public double getEncoderPosition() {
    return maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).getPosition();
  }

  @Override
  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).getForwardLimitSwitch();
  }

  @Override
  public boolean getLimitSwitchRValuse() {
    return maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).getReversLimitSwitch();
  }

  @Override
  public void resetSensor() {
    maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).overrideLimitSwitches(overrid);
  }

  @Override
  public double calculatePIDOutput(double setPoint) {
    return ElevatorMovePID.calculate(getEncoderPosition(), setPoint);
  }

  @Override
  public boolean isPIDAtTarget(double waitTime) {

    return ElevatorMovePID.atSetpoint(waitTime);
  }

  @Override
  public double getSetpointPID() {
    return ElevatorMovePID.getSetpoint();
  }

  @Override
  public double getPositionError() {
    return ElevatorMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
    ElevatoShuffleboard.addNum("ElevatorGetPosition", getEncoderPosition());
    ElevatoShuffleboard.addNum("ElevatorGetSetPoint", getSetpointPID());
    ElevatoShuffleboard.addNum("ElevatorGetPositionError", getPositionError());
    ElevatoShuffleboard.addBoolean("ElevatorAtSetPoint", isPIDAtTarget(0.1));
    ElevatoShuffleboard.addBoolean("ElevatorGetLimitSwitchRValuse", getLimitSwitchRValuse());
    ElevatoShuffleboard.addBoolean("ElevatorGetLimitSwitchFValuse", getLimitSwitchFValuse());
  }

  public static Elevator getinstance() {
    if (m_Elevator == null) {
      m_Elevator = new Elevator();
    }
    return m_Elevator;
  }
}
