// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Arm extends SubsystemBase implements MASubsystem {

  private MAMotorControler ArmMove;
  private MAPidController ArmMovePID;
  private static Arm m_Arm;
  private MAShuffleboard ArmShuffleBoard = new MAShuffleboard(ArmConstants.KSUBSYSTEM_NAME);

  private Arm() {
    ArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(ArmMove);

    // cahnge Tolorance
    ArmMovePID = new MAPidController(ArmConstants.KP_ARM_MOVE, ArmConstants.KI_ARM_MOVE, ArmConstants.KD_ARM_MOVE,
        ArmConstants.KF_ARM_MOVE, 10, -12, 12);
    resetEncoder();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * set voltage -12 to 12 indax 0 - ArmMove
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(Indax).setVoltage(Power);
  }

  public double getEncoderPosition() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getPosition();
  }

  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getForwardLimitSwitch();
  }

  public boolean getLimitSwitchRValuse() {
    return maMotorControlers.get(ArmConstants.KARM_MOVE).getReversLimitSwitch();
  }

  public void resetEncoder() {
    maMotorControlers.get(ArmConstants.KARM_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ArmConstants.KARM_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateArmMovePID(double setPoint) {
    return ArmMovePID.calculate(getEncoderPosition(), setPoint); // can be void and set diracle to the motor
  }

  public boolean isArmMovePIDAtTarget(double waitTime) {
    return ArmMovePID.atSetpoint(waitTime);// can be void and set diracle to the motor
  }

  public double getSetpointArmMovePID() {
    return ArmMovePID.getSetpoint();
  }

  public double getPositionErrorArmMovePID() {
    return ArmMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
    ArmShuffleBoard.addNum("getPosition", getEncoderPosition());
    ArmShuffleBoard.addNum("getSetPoint", getSetpointArmMovePID());
    ArmShuffleBoard.addNum("getPositionError", getPositionErrorArmMovePID());
    ArmShuffleBoard.addBoolean("atSetPoint", isArmMovePIDAtTarget(0.1));
    ArmShuffleBoard.addBoolean("getLimitSwitchRValuse", getLimitSwitchRValuse());
    ArmShuffleBoard.addBoolean("getLimitSwitchFValuse", getLimitSwitchFValuse());
  }

  public static Arm getinstance() {
    if (m_Arm == null) {
      m_Arm = new Arm();
    }
    return m_Arm;
  }
}
