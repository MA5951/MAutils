// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;

public class Elevator extends SubsystemBase implements MASubsystem {

  private MAMotorControler ElevatorMove;
  private MAPidController ElevatorMovePID;
  private static Elevator m_Elevator;
  private static final int ELEVATOR_MOVE = 0;
  private static final double KP_ELEVATOR_MOVE = 0; // TODO
  private static final double KI_ELEVATOR_MOVE = 0; // TODO
  private static final double KD_ELEVATOR_MOVE = 0; // TODO
  private static final double KF_ELEVATOR_MOVE = 0; // TODO

  private Elevator() {
    ElevatorMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, true, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(ElevatorMove);
    // addMAMotorComtrolers(ElevatorMove, IDMotor.ID3); if have more then one motor

    // cahnge Tolorance
    ElevatorMovePID = new MAPidController(KP_ELEVATOR_MOVE, KI_ELEVATOR_MOVE, KD_ELEVATOR_MOVE, KF_ELEVATOR_MOVE, 10,
        -12, 12);
    resetEncoder();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * set voltage -12 to 12
   * indax 0 - ElevatorMove 
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(Indax).setvoltage(Power);
  }

  public double getPosition() {
    return maMotorControlers.get(ELEVATOR_MOVE).getPosition();
  }

  public boolean getLimitSwitchFValuse() {
    return maMotorControlers.get(ELEVATOR_MOVE).getForwardLimitSwitch(); // change the Limit
  }

  public boolean getLimitSwitchRValuse() {
    return maMotorControlers.get(ELEVATOR_MOVE).getReversLimitSwitch(); // change the Limit
  }

  public void resetEncoder() {
    maMotorControlers.get(ELEVATOR_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(ELEVATOR_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateElevatorPID(double setPoint) {
    return ElevatorMovePID.calculate(getPosition(), setPoint); // can be void and set diracle to the motor
  }

  public boolean isElevatorPIDAtTarget(double waitTime) {
    return ElevatorMovePID.atSetpoint(waitTime);// can be void and set diracle to the motor
  }

  public double getSetpointElevatorPID() {
    return ElevatorMovePID.getSetpoint();
  }

  public double getPositionErrorElevatorPID() {
    return ElevatorMovePID.getPositionError();
  }

  @Override
  public void PrintValues() {

  }





  public static Elevator getinstance() {
    if (m_Elevator == null) {
      m_Elevator = new Elevator();
    }
    return m_Elevator;
  }
}
