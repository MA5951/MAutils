// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;

public class TwoJointedArm extends SubsystemBase implements MASubsystem {
  private MAMotorControler ArmMove;
  private MAPidController ArmMovePID;
  private MAMotorControler SecArmMove;
  private MAPidController SecArmMovePID;

  private static TwoJointedArm m_TwoJointedArm;
  private static final int ARM_MOVE = 0;
  private static final int SEC_ARM_MOVE = 0;

  private static final double KP_ARM_MOVE = 0; // TODO
  private static final double KI_ARM_MOVE = 0; // TODO
  private static final double KD_ARM_MOVE = 0; // TODO
  private static final double KF_ARM_MOVE = 0; // TODO

  private static final double KP_SEC_ARM_MOVE = 0; // TODO
  private static final double KI_SEC_ARM_MOVE = 0; // TODO
  private static final double KD_SEC_ARM_MOVE = 0; // TODO
  private static final double KF_SEC_ARM_MOVE = 0; // TODO

  private TwoJointedArm() {
    ArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(ArmMove);
    // addMAMotorComtrolers(ElevatorMove, IDMotor.ID3); if have more then one motor
    // cahnge Tolorance
    ArmMovePID = new MAPidController(KP_ARM_MOVE, KI_ARM_MOVE, KD_ARM_MOVE, KF_ARM_MOVE, 10, -12, 12);

    SecArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID3, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(SecArmMove);
    // cahnge Tolorance
    SecArmMovePID = new MAPidController(KP_SEC_ARM_MOVE, KI_SEC_ARM_MOVE, KD_SEC_ARM_MOVE, KF_SEC_ARM_MOVE, 10, -12,
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
    maMotorControlers.get(ARM_MOVE).resetEncoder();
    maMotorControlers.get(SEC_ARM_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ARM_MOVE).overrideLimitSwitches(overrid);
    maMotorControlers.get(SEC_ARM_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateArmMovePID(double setPoint) {
    return ArmMovePID.calculate(getPosition(ARM_MOVE), setPoint); // can be void and set diracle to the motor
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
    return SecArmMovePID.calculate(getPosition(SEC_ARM_MOVE), setPoint); // can be void and set diracle to the motor
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
  
  }

  public static TwoJointedArm getinstance() {
    if (m_TwoJointedArm == null) {
      m_TwoJointedArm = new TwoJointedArm();
    }
    return m_TwoJointedArm;
  }
}
