// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MotorIntake extends SubsystemBase implements MASubsystem {

  private MAMotorControler IntakeMove;
  private MAMotorControler IntakeCollection;
  private MAPidController IntakeMovePID;
  private static MotorIntake m_Intake;

  private MAShuffleboard motorIntakesShuffleboard = new MAShuffleboard(IntakeConstants.SubsystemName);

  private MotorIntake() {

    IntakeCollection = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID1);
    setMAMotorComtrolers(IntakeCollection);

    // change the Limit
    IntakeMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, false, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(IntakeMove);

    // cahnge Tolorance
    IntakeMovePID = new MAPidController(IntakeConstants.KP_INTAKE_MOVE, IntakeConstants.KI_INTAKE_MOVE,
        IntakeConstants.KD_INTAKE_MOVE, 0, 10, -1, 1);
    resetEncoder();
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  /**
   * indax 0 - IntakeCollection indax 1 - IntakeMove
   */
  @Override
  public Runnable setMotorPower(double Power, int Indax) {
    return () -> maMotorControlers.get(Indax).set(Power);
  }

  public double getPosition() {
    return maMotorControlers.get(IntakeConstants.INTAKE_MOVE).getPosition();
  }

  public boolean getLimitSwitchValuse() {
    return maMotorControlers.get(IntakeConstants.INTAKE_MOVE).getForwardLimitSwitch(); // change the Limit

  }

  public void resetEncoder() {
    maMotorControlers.get(IntakeConstants.INTAKE_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(IntakeConstants.INTAKE_MOVE).overrideLimitSwitches(overrid);
  }

  public double calculateIntakeMovePID(double setPoint) {
    return IntakeMovePID.calculate(getPosition(), setPoint); // can be void and set diracle to the motor
  }

  public boolean isIntakeMovePIDAtTarget(double waitTime) {
    return IntakeMovePID.atSetpoint(waitTime);// can be void and set diracle to the motor
  }

  public double getSetpointIntakeMovePID() {
    return IntakeMovePID.getSetpoint();
  }

  public double getPositionErrorIntakeMovePID() {
    return IntakeMovePID.getPositionError();
  }

  public double getStatorCurrent(int indax) {
    return maMotorControlers.get(indax).getStatorCurrent();
  }

  @Override
  public void PrintValues() {
    motorIntakesShuffleboard.addNum("getPosition", getPosition());
    motorIntakesShuffleboard.addNum("getSetPoint", getSetpointIntakeMovePID());
    motorIntakesShuffleboard.addNum("getPositionError", getPositionErrorIntakeMovePID());
    motorIntakesShuffleboard.addNum("getStatorCurrentMovetMotor", getStatorCurrent(IntakeConstants.INTAKE_MOVE));
    motorIntakesShuffleboard.addNum("getStatorCurrentCollection", getStatorCurrent(IntakeConstants.INTAKE_COLLECTION));
    motorIntakesShuffleboard.addBoolean("atSetPoint", isIntakeMovePIDAtTarget(0.1));
    motorIntakesShuffleboard.addBoolean("LimitSwitchValuse", getLimitSwitchValuse());
  }

  public static MotorIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new MotorIntake();
    }
    return m_Intake;
  }
}
