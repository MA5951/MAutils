// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;

public class MotorIntake extends SubsystemBase implements MASubsystem {

  private MAMotorControler IntakeMove;
  private MAMotorControler IntakeCollection;
  private MAPidController IntakeMovePID;
  private static MotorIntake m_Intake;
  private static final int INTAKE_MOVE = 1;
  private static final int INTAKE_COLLECTION = 0;
  private static final double KP_INTAKE_MOVE = 0; // TODO
  private static final double KI_INTAKE_MOVE = 0; // TODO
  private static final double KD_INTAKE_MOVE = 0; // TODO

  private MotorIntake() {

    IntakeCollection = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID1);
    setMAMotorComtrolers(IntakeCollection);

    // change the Limit
    IntakeMove = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID2, false, 0, false, true, true, ENCODER.Encoder);
    setMAMotorComtrolers(IntakeMove);
    //addMAMotorComtrolers(IntakeMove, IDMotor.ID3); if have more then one motor 
    
    // cahnge Tolorance
    IntakeMovePID = new MAPidController(KP_INTAKE_MOVE, KI_INTAKE_MOVE, KD_INTAKE_MOVE, 0, 10, -1, 1);
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
    return maMotorControlers.get(INTAKE_MOVE).getPosition();
  }

  public boolean getLimitSwitchValuse() {
    return maMotorControlers.get(INTAKE_MOVE).getForwardLimitSwitch(); // change the Limit

  }

  public void resetEncoder() {
    maMotorControlers.get(INTAKE_MOVE).resetEncoder();
  }

  public void overrideLimitSwitches(boolean overrid) {
    maMotorControlers.get(INTAKE_MOVE).overrideLimitSwitches(overrid);
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

  @Override
  public void PrintValues() {

  }



  

  public static MotorIntake getinstance() {
    if (m_Intake == null) {
      m_Intake = new MotorIntake();
    }
    return m_Intake;
  }
}
