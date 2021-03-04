// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Arm extends MASubsystem {

  private MAMotorControler ArmMove;
  private MAPidController ArmMovePID;
  private static Arm m_Arm;
  private MAShuffleboard ArmShuffleBoard = new MAShuffleboard(ArmConstants.KSUBSYSTEM_NAME);

  private Arm() {
    ArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID5, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(ArmMove);
    // cahnge Tolorance
    ArmMovePID = new MAPidController(ArmConstants.KP_ARM_MOVE, ArmConstants.KI_ARM_MOVE, ArmConstants.KD_ARM_MOVE,
        ArmConstants.KF_ARM_MOVE, 10, -12, 12);
    resetSensor();
   
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * set voltage -12 to 12 indax 0 - ArmMove
   */
  @Override
  public void setMotorPower(double Power, int Indax) {
    maMotorControlers.get(Indax).setVoltage(Power);
  }

  @Override
  public double getEncoderPosition() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getPosition();
  }

  @Override
  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getForwardLimitSwitch();
  }

  @Override
  public boolean getLimitSwitchRValuse() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getReversLimitSwitch();
  }

  @Override
  public void resetSensor() {
    maMotorControlers.get(ArmConstants.KARM_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ArmConstants.KARM_MOVE).overrideLimitSwitches(overrid);
  }

  @Override
  public double calculatePIDOutput(double setPoint) {
    return ArmMovePID.calculate(getEncoderPosition(), setPoint); // can be void and set diracle to the motor
  }

  @Override
  public boolean isPIDAtTarget(double waitTime) {
    return ArmMovePID.atSetpoint(waitTime);// can be void and set diracle to the motor
  }

  @Override
  public double getSetpointPID() {
    return ArmMovePID.getSetpoint();
  }

  @Override
  public double getPositionError() {
    return ArmMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
    ArmShuffleBoard.addNum("ARMgetPosition", getEncoderPosition());
    ArmShuffleBoard.addNum("ARMgetSetPoint", getSetpointPID());
    ArmShuffleBoard.addNum("ARMgetPositionError", getPositionError());
    ArmShuffleBoard.addBoolean("ARMatSetPoint", isPIDAtTarget(0.1));
    ArmShuffleBoard.addBoolean("ARMgetLimitSwitchRValuse", getLimitSwitchRValuse());
    ArmShuffleBoard.addBoolean("ARMgetLimitSwitchFValuse", getLimitSwitchFValuse());
  }

  public static Arm getinstance() {
    if (m_Arm == null) {
      m_Arm = new Arm();
    }
    return m_Arm;
  }
}
