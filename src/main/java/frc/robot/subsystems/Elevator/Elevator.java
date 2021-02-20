// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Elevator extends SubsystemBase implements MASubsystem {

  private MAMotorControler ElevatorMove;
  private MAPidController ElevatorMovePID;
  private static Elevator m_Elevator;
  private MAShuffleboard ElevatoShuffleboard = new MAShuffleboard(ElevatorConstants.KSUBSYSTEM_NAME);

  private Elevator() {
    ElevatorMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(ElevatorMove);

    // cahnge Tolorance
    ElevatorMovePID = new MAPidController(ElevatorConstants.KP_ELEVATOR_MOVE, ElevatorConstants.KI_ELEVATOR_MOVE,
        ElevatorConstants.KD_ELEVATOR_MOVE, ElevatorConstants.KF_ELEVATOR_MOVE, 10, -12, 12);
    resetEncoder();
  }

  @Override
  public void periodic() {
    PrintValues();
  }

  /**
   * set voltage -12 to 12 indax 0 - ElevatorMove
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(Indax).setVoltage(Power);
  }

  public double getEncoderPosition() {
    return maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).getPosition();
  }

  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).getForwardLimitSwitch();
  }

  public boolean getLimitSwitchRValuse() {
    return maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).getReversLimitSwitch();
  }

  public void resetEncoder() {
    maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ElevatorConstants.ELEVATOR_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateElevatorPID(double setPoint) {
    return ElevatorMovePID.calculate(getEncoderPosition(), setPoint);
  }

  public boolean isElevatorPIDAtTarget(double waitTime) {
    return ElevatorMovePID.atSetpoint(waitTime);
  }

  public double getSetpointElevatorPID() {
    return ElevatorMovePID.getSetpoint();
  }

  public double getPositionErrorElevatorPID() {
    return ElevatorMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
    ElevatoShuffleboard.addNum("getPosition", getEncoderPosition());
    ElevatoShuffleboard.addNum("getSetPoint", getSetpointElevatorPID());
    ElevatoShuffleboard.addNum("getPositionError", getPositionErrorElevatorPID());
    ElevatoShuffleboard.addBoolean("atSetPoint", isElevatorPIDAtTarget(0.1));
    ElevatoShuffleboard.addBoolean("getLimitSwitchRValuse", getLimitSwitchRValuse());
    ElevatoShuffleboard.addBoolean("getLimitSwitchFValuse", getLimitSwitchFValuse());
  }

  public static Elevator getinstance() {
    if (m_Elevator == null) {
      m_Elevator = new Elevator();
    }
    return m_Elevator;
  }
}
