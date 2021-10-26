// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControllers.MAMotorController;
import frc.robot.utils.controllers.MAPidController;
import frc.robot.utils.controllers.interfaces.Controler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Arm extends MASubsystem {

  private MAMotorController armMove;
  private Controler armMovePID;
  private static Arm m_Arm;
  private MAShuffleboard armShuffleBoard = new MAShuffleboard(ArmConstants.KSUBSYSTEM_NAME);

  private Arm() {
    armMove = new MAMotorController(MOTOR_CONTROLL.TALON, ID5, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(armMove);
    // cahnge Tolerance
    armMovePID = new MAPidController(ArmConstants.KP_ARM_MOVE, ArmConstants.KI_ARM_MOVE, ArmConstants.KD_ARM_MOVE, 0,
        10, -12, 12);
    resetSensor();

  }

  @Override
  public void periodic() {
    printValues();

  }

  /**
   * set voltage -12 to 12 index 0 - ArmMove
   */
  @Override
  public void setMotorPower(double Power, int Index) {
    maMotorControlers.get(Index).setVoltage(Power);
  }

  @Override
  public double getEncoderPosition() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getPosition();
  }

  @Override
  public boolean getLimitSwitchFValues() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getForwardLimitSwitch();
  }

  @Override
  public boolean getLimitSwitchRValues() {
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
    return armMovePID.calculate(getEncoderPosition(), setPoint); //TODO add F
  }

  @Override
  public boolean isPIDAtTarget() {
    return armMovePID.atSetpoint();
  }

  @Override
  public double getSetpointPID() {
    return armMovePID.getSetpoint();
  }

  @Override
  public double getPositionError() {
    return armMovePID.getPositionError();
  }

  @Override
  public void printValues() {
    armShuffleBoard.addNum("ARMgetPosition", getEncoderPosition());
    armShuffleBoard.addNum("ARMgetSetPoint", getSetpointPID());
    armShuffleBoard.addNum("ARMgetPositionError", getPositionError());
    armShuffleBoard.addBoolean("ARMatSetPoint", isPIDAtTarget());
    armShuffleBoard.addBoolean("ARMgetLimitSwitchRValues", getLimitSwitchRValues());
    armShuffleBoard.addBoolean("ARMgetLimitSwitchFValues", getLimitSwitchFValues());
  }

  public static Arm getinstance() {
    if (m_Arm == null) {
      m_Arm = new Arm();
    }
    return m_Arm;
  }
}
