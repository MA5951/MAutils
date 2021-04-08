// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Actuators.MAPiston;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Elevator extends MASubsystem {

  private MAMotorControler elevatorMove;
  private MAPidController elevatorMovePID;
  private MAPiston piston;
  private static Elevator m_Elevator;
  private MAShuffleboard elevatoShuffleboard = new MAShuffleboard(ElevatorConstants.KSUBSYSTEM_NAME);

  private Elevator() {
    piston = new MAPiston(RobotConstants.P_ID6);
    elevatorMove = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID8, false, 0, true,
        ENCODER.Alternate_Encoder);
    setMAMotorComtrolersList(elevatorMove);

    // cahnge Tolorance
    elevatorMovePID = new MAPidController(ElevatorConstants.KP_ELEVATOR_MOVE, ElevatorConstants.KI_ELEVATOR_MOVE,
        ElevatorConstants.KD_ELEVATOR_MOVE, ElevatorConstants.KF_ELEVATOR_MOVE, 10, -12, 12);
    resetSensor();

  }

  @Override
  public void periodic() {
    printValues();
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
    return maMotorControlers.get(ElevatorConstants.KELEVATOR_MOVE).getPosition();
  }

  @Override
  public void resetSensor() {
    maMotorControlers.get(ElevatorConstants.KELEVATOR_MOVE).resetEncoder();
  }

  @Override
  public double calculatePIDOutput(double setPoint) {
    return elevatorMovePID.calculate(getEncoderPosition(), setPoint);
  }

  @Override
  public boolean isPIDAtTarget() {
    return elevatorMovePID.atSetpoint();
  }

  @Override
  public double getSetpointPID() {
    return elevatorMovePID.getSetpoint();
  }

  @Override
  public double getPositionError() {
    return elevatorMovePID.getPositionError();
  }

  @Override
  public void togglePiston() {
    piston.toggle();
  }

  @Override
  public boolean getPistonValue() {

    return piston.get();
  }

  @Override
  public void printValues() {
    elevatoShuffleboard.addNum("ElevatorGetPosition", getEncoderPosition());
    elevatoShuffleboard.addNum("ElevatorGetSetPoint", getSetpointPID());
    elevatoShuffleboard.addNum("ElevatorGetPositionError", getPositionError());
    elevatoShuffleboard.addBoolean("ElevatorAtSetPoint", isPIDAtTarget());
    elevatoShuffleboard.addBoolean("ElevatorGetLimitSwitchRValuse", getLimitSwitchRValuse());
    elevatoShuffleboard.addBoolean("ElevatorGetLimitSwitchFValuse", getLimitSwitchFValuse());
  }

  public static Elevator getinstance() {
    if (m_Elevator == null) {
      m_Elevator = new Elevator();
    }
    return m_Elevator;
  }
}
