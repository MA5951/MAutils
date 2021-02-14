// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MoonShooter extends SubsystemBase implements MASubsystem {
  private MAMotorControler TransformMotor;
  private MAMotorControler MotorA;
  private MAPidController pidControllerMotorA;
  private final static double MOTOR_A_KP = 0;
  private final static double MOTOR_A_KI = 0;
  private final static double MOTOR_A_KD = 0;
  private final static int MOTOR_A = 0;
  private final static int TRANSFORM_MOTOR = 1;
  private static MoonShooter MoonShooter;
  private MAShuffleboard moonShootersShuffleboard = new MAShuffleboard(""); // TODO

  private MoonShooter() {
    MotorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolers(MotorA);
    // cahnge the tolorance
    pidControllerMotorA = new MAPidController(MOTOR_A_KP, MOTOR_A_KI, MOTOR_A_KD, 0, 10, -12, 12);

    TransformMotor = new MAMotorControler(MOTOR_CONTROLL.VICTOR, IDMotor.ID3);
    setMAMotorComtrolers(TransformMotor);
  }

  @Override
  public void periodic() {
    PrintValues();
  }

  public double getTransformMotorStatorCurrent() {
    return maMotorControlers.get(TRANSFORM_MOTOR).getStatorCurrent(); // dont work with victor(use the PDP)
  }

  /**
   * voltage -12 to 12, MotorA = 0 , TransformMotor = 1
   */
  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).setvoltage(power);
  }

  public double getMotorARPM() {
    return maMotorControlers.get(MOTOR_A).getVelocity(); // now not retunr RPM
  }

  public double calculatePIDOutPutMotorA(double setPoint) {
    return pidControllerMotorA.calculate(getMotorARPM(), setPoint);
  }

  public boolean isAtSetPointMotorA(double waitTime) {
    return pidControllerMotorA.atSetpoint(waitTime);
  }

  public void setSetPointMotorA(double setPoint) {
    pidControllerMotorA.setF(0); // TODO
    pidControllerMotorA.setSetpoint(setPoint);
  }

  public double DistanceToRPM(double distance) {
    return 0; // TODO
  }

  @Override
  public void PrintValues() {
    moonShootersShuffleboard.addNum("MotorA RPM", getMotorARPM());
    moonShootersShuffleboard.addNum("MotorA PIDSetPoint", pidControllerMotorA.getSetpoint());
    moonShootersShuffleboard.addNum("MotorA PositionError", pidControllerMotorA.getPositionError());
    moonShootersShuffleboard.addBoolean("MotorA AtSetPoint", isAtSetPointMotorA(0.1));
    moonShootersShuffleboard.addNum("getTransformMotorStatorCurrent", getTransformMotorStatorCurrent());
  }

  public static MoonShooter getinstance() {
    if (MoonShooter == null) {
      MoonShooter = new MoonShooter();
    }
    return MoonShooter;
  }
}
