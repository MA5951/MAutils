// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.controlers.MAPidController;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class TwoJointedArm extends SubsystemBase implements MASubsystem {
  private MAMotorControler ArmMove;
  private MAPidController ArmMovePID;
  private MAMotorControler SecArmMove;
  private MAPidController SecArmMovePID;

  private static TwoJointedArm m_TwoJointedArm;
  private MAShuffleboard TwoJointedArm = new MAShuffleboard(ArmConstants.KSUBSYSTEM_NAME);

  private TwoJointedArm() {
    ArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(ArmMove);

    // cahnge Tolorance
    ArmMovePID = new MAPidController(ArmConstants.KP_ARM_MOVE, ArmConstants.KI_ARM_MOVE, ArmConstants.KD_ARM_MOVE,
        ArmConstants.KF_ARM_MOVE, 10, -12, 12);

    SecArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID3, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolersList(SecArmMove);
    // cahnge Tolorance
    SecArmMovePID = new MAPidController(ArmConstants.KP_SEC_ARM_MOVE, ArmConstants.KI_SEC_ARM_MOVE,
        ArmConstants.KD_SEC_ARM_MOVE, ArmConstants.KF_SEC_ARM_MOVE, 10, -12, 12);
    resetArmEncoder();
    resetSecArmEncoder();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * set voltage -12 to 12 indax 0 - ArmMove , indax 1 - SecArmMove
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(Indax).setVoltage(Power);
  }

  public double getEncoderPosition(int Motor) {
    return maMotorControlers.get(Motor).getPosition();
  }

  public boolean getLimitSwitchFValuse(int Motor) {
    return maMotorControlers.get(Motor).getForwardLimitSwitch(); // change the Limit
  }

  public boolean getLimitSwitchRValuse(int Motor) {
    return maMotorControlers.get(Motor).getReversLimitSwitch(); // change the Limit
  }

  public void resetArmEncoder() {
    maMotorControlers.get(ArmConstants.KARM_MOVE).resetEncoder();
  }

  public void resetSecArmEncoder() {
    maMotorControlers.get(ArmConstants.KSEC_ARM_MOVE).resetEncoder();
  }

  public void overrideSecArmLimitSwitches(boolean overrid) {
    maMotorControlers.get(ArmConstants.KSEC_ARM_MOVE).overrideLimitSwitches(overrid);
  }

  public void overrideArmLimitSwitches(boolean overrid) {
    maMotorControlers.get(ArmConstants.KARM_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateArmMovePID(double setPoint) {
    return ArmMovePID.calculate(getEncoderPosition(ArmConstants.KARM_MOVE), setPoint);
  }

  public boolean isArmMovePIDAtTarget(double waitTime) {
    return ArmMovePID.atSetpoint(waitTime);
  }

  public double getSetpointArmMovePID() {
    return ArmMovePID.getSetpoint();
  }

  public double getPositionErrorArmMovePID() {
    return ArmMovePID.getPositionError();
  }

  public double calculateSecArmMovePID(double setPoint) {
    return SecArmMovePID.calculate(getEncoderPosition(ArmConstants.KSEC_ARM_MOVE), setPoint);
  }

  public boolean isSecArmMovePIDAtTarget(double waitTime) {
    return SecArmMovePID.atSetpoint(waitTime);
  }

  public double getSetpointSecArmMovePID() {
    return SecArmMovePID.getSetpoint();
  }

  public double getPositionErrorSecArmMovePID() {
    return SecArmMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
    TwoJointedArm.addNum("ArmMovegetPosition", getEncoderPosition(ArmConstants.KARM_MOVE));
    TwoJointedArm.addNum("SecArmMovegetPosition", getEncoderPosition(ArmConstants.KSEC_ARM_MOVE));

    TwoJointedArm.addNum("ArmMovegetSetPoint", getSetpointArmMovePID());
    TwoJointedArm.addNum("ArmMovegetPositionError", getPositionErrorArmMovePID());
    TwoJointedArm.addBoolean("ArmMoveatSetPoint", isArmMovePIDAtTarget(0.1));

    TwoJointedArm.addNum("SecArmMovegetSetPoint", getSetpointSecArmMovePID());
    TwoJointedArm.addNum("SecArmMovegetPositionError", getPositionErrorSecArmMovePID());
    TwoJointedArm.addBoolean("SecArmMoveatSetPoint", isSecArmMovePIDAtTarget(0.1));

    TwoJointedArm.addBoolean("getArmMoveLimitSwitchFValuse", getLimitSwitchFValuse(ArmConstants.KARM_MOVE));
    TwoJointedArm.addBoolean("getArmMoveLimitSwitchRValuse", getLimitSwitchRValuse(ArmConstants.KARM_MOVE));

    TwoJointedArm.addBoolean("getSecArmMoveLimitSwitchFValuse", getLimitSwitchFValuse(ArmConstants.KSEC_ARM_MOVE));
    TwoJointedArm.addBoolean("getSecArmMoveLimitSwitchRValuse", getLimitSwitchRValuse(ArmConstants.KSEC_ARM_MOVE));
  }

  public static TwoJointedArm getinstance() {
    if (m_TwoJointedArm == null) {
      m_TwoJointedArm = new TwoJointedArm();
    }
    return m_TwoJointedArm;
  }
}
