// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class TwoJointedArm extends SubsystemBase implements MASubsystem {
  private MAMotorControler ArmMove;
  private MAPidController ArmMovePID;
  private MAMotorControler SecArmMove;
  private MAPidController SecArmMovePID;

  private static TwoJointedArm m_TwoJointedArm;
  private MAShuffleboard TwoJointedArm = new MAShuffleboard(""); // TODO

  private TwoJointedArm() {
    ArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(ArmMove);
    // addMAMotorComtrolers(ElevatorMove, IDMotor.ID3); if have more then one motor
    // cahnge Tolorance
    ArmMovePID = new MAPidController(ArmConstants.KP_ARM_MOVE, ArmConstants.KI_ARM_MOVE, ArmConstants.KD_ARM_MOVE, ArmConstants.KF_ARM_MOVE, 10, -12, 12);

    SecArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID3, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(SecArmMove);
    // cahnge Tolorance
    SecArmMovePID = new MAPidController(ArmConstants.KP_SEC_ARM_MOVE, ArmConstants.KI_SEC_ARM_MOVE, ArmConstants.KD_SEC_ARM_MOVE, ArmConstants.KF_SEC_ARM_MOVE, 10, -12,
        12);
    resetEncoder();
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
    return () -> maMotorControlers.get(Indax).setvoltage(Power);
  }

  public double getPosition(int Motor) {
    return maMotorControlers.get(Motor).getPosition();
  }

  public boolean getLimitSwitchFValuse(int Motor) {
    return maMotorControlers.get(Motor).getForwardLimitSwitch(); // change the Limit
  }

  public boolean getLimitSwitchRValuse(int Motor) {
    return maMotorControlers.get(Motor).getReversLimitSwitch(); // change the Limit
  }

  public void resetEncoder() {
    maMotorControlers.get(ArmConstants.ARM_MOVE).resetEncoder();
    maMotorControlers.get(ArmConstants.SEC_ARM_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ArmConstants.ARM_MOVE).overrideLimitSwitches(overrid);
    maMotorControlers.get(ArmConstants.SEC_ARM_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateArmMovePID(double setPoint) {
    return ArmMovePID.calculate(getPosition(ArmConstants.ARM_MOVE), setPoint); // can be void and set diracle to the motor
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

  public double calculateSecArmMovePID(double setPoint) {
    return SecArmMovePID.calculate(getPosition(ArmConstants.SEC_ARM_MOVE), setPoint); // can be void and set diracle to the motor
  }

  public boolean isSecArmMovePIDAtTarget(double waitTime) {
    return SecArmMovePID.atSetpoint(waitTime);// can be void and set diracle to the motor
  }

  public double getSetpointSecArmMovePID() {
    return SecArmMovePID.getSetpoint();
  }

  public double getPositionErrorSecArmMovePID() {
    return SecArmMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
    TwoJointedArm.addNum("ArmMovegetPosition", getPosition(ArmConstants.ARM_MOVE));
    TwoJointedArm.addNum("SecArmMovegetPosition", getPosition(ArmConstants.SEC_ARM_MOVE));

    TwoJointedArm.addNum("ArmMovegetSetPoint", getSetpointArmMovePID());
    TwoJointedArm.addNum("ArmMovegetPositionError", getPositionErrorArmMovePID());
    TwoJointedArm.addBoolean("ArmMoveatSetPoint", isArmMovePIDAtTarget(0.1));

    TwoJointedArm.addNum("SecArmMovegetSetPoint", getSetpointSecArmMovePID());
    TwoJointedArm.addNum("SecArmMovegetPositionError", getPositionErrorSecArmMovePID());
    TwoJointedArm.addBoolean("SecArmMoveatSetPoint", isSecArmMovePIDAtTarget(0.1));

    TwoJointedArm.addBoolean("getArmMoveLimitSwitchFValuse", getLimitSwitchFValuse(ArmConstants.ARM_MOVE));
    TwoJointedArm.addBoolean("getArmMoveLimitSwitchRValuse", getLimitSwitchRValuse(ArmConstants.ARM_MOVE));

    TwoJointedArm.addBoolean("getSecArmMoveLimitSwitchFValuse", getLimitSwitchFValuse(ArmConstants.SEC_ARM_MOVE));
    TwoJointedArm.addBoolean("getSecArmMoveLimitSwitchRValuse", getLimitSwitchRValuse(ArmConstants.SEC_ARM_MOVE));
  }

  public static TwoJointedArm getinstance() {
    if (m_TwoJointedArm == null) {
      m_TwoJointedArm = new TwoJointedArm();
    }
    return m_TwoJointedArm;
  }
}
