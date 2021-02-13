// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;

public class Arm extends SubsystemBase implements MASubsystem {

  private MAMotorControler ArmMove;
  private MAPidController ArmMovePID;
  private static Arm m_Arm;
  private static final int ARM_MOVE = 0;
  private static final double KP_ARM_MOVE = 0; // TODO
  private static final double KI_ARM_MOVE = 0; // TODO
  private static final double KD_ARM_MOVE = 0; // TODO
  private static final double KF_ARM_MOVE = 0; // TODO

  private Arm() {
    ArmMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(ArmMove);
    // addMAMotorComtrolers(ElevatorMove, IDMotor.ID3); if have more then one motor

    // cahnge Tolorance
    ArmMovePID = new MAPidController(KP_ARM_MOVE, KI_ARM_MOVE, KD_ARM_MOVE, KF_ARM_MOVE, 10, -12, 12);
    resetEncoder();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * set voltage -12 to 12
   * indax 0 - ArmMove 
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(Indax).setvoltage(Power);
  }

  public double getPosition() {
    return maMotorControlers.get(ARM_MOVE).getPosition();
  }

  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(ARM_MOVE).getForwardLimitSwitch(); // change the Limit
  }

  public boolean getLimitSwitchRValuse() {
    return maMotorControlers.get(ARM_MOVE).getReversLimitSwitch(); // change the Limit
  }

  public void resetEncoder() {
    maMotorControlers.get(ARM_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ARM_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateIntakeMovePID(double setPoint) {
    return ArmMovePID.calculate(getPosition(), setPoint); // can be void and set diracle to the motor
  }

  public boolean isIntakeMovePIDAtTarget(double waitTime) {
    return ArmMovePID.atSetpoint(waitTime);// can be void and set diracle to the motor
  }

  public double getSetpointIntakeMovePID() {
    return ArmMovePID.getSetpoint();
  }

  public double getPositionErrorIntakeMovePID() {
    return ArmMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {
 
  }

 

 
  public static Arm getinstance() {
    if (m_Arm == null) {
      m_Arm = new Arm();
    }
    return m_Arm;
  }
}
