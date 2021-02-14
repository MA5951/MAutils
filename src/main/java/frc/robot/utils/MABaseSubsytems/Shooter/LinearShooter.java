// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MABaseSubsytems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class LinearShooter extends SubsystemBase implements MASubsystem {
  private MAMotorControler TransformMotor;

  private MAMotorControler MotorA;
  private MAPidController pidControllerMotorA;
  private final static double MOTOR_A_KP = 0;
  private final static double MOTOR_A_KI = 0;
  private final static double MOTOR_A_KD = 0;

  private MAMotorControler MotorB;
  private MAPidController pidControllerMotorB;
  private final static double MOTOR_B_KP = 0;
  private final static double MOTOR_B_KI = 0;
  private final static double MOTOR_B_KD = 0;

  private final static int MOTOR_A = 0;
  private final static int MOTOR_B = 1;
  private final static int TRANSFORM_MOTOR = 2;

  private static LinearShooter LinearShooter;
  private MAShuffleboard linearShooterShuffleboard = new MAShuffleboard(""); // TODO

  private LinearShooter() {
    TransformMotor = new MAMotorControler(MOTOR_CONTROLL.VICTOR, IDMotor.ID3);
    setMAMotorComtrolers(TransformMotor);

    MotorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolers(MotorA);
    pidControllerMotorA = new MAPidController(MOTOR_A_KP, MOTOR_A_KI, MOTOR_A_KD, 0, 10, -12, 12); // cahnge the
                                                                                                   // tolorance

    MotorB = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID2, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolers(MotorB);
    pidControllerMotorB = new MAPidController(MOTOR_B_KP, MOTOR_B_KI, MOTOR_B_KD, 0, 10, -12, 12); // cahnge the
                                                                                                   // tolorance
  }

  @Override
  public void periodic() {
    PrintValues();
  }

  public double getTransformMotorStatorCurrent() {
    return maMotorControlers.get(TRANSFORM_MOTOR).getStatorCurrent(); // dont work with victor;
  }

  /**
   * voltage -12 to 12, MotorA = 0 , MotorB = 1 , TransformMotor = 2
   */
  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).setvoltage(power);
  }

  public double getMotorARPM(int Indax) {
    return maMotorControlers.get(Indax).getVelocity(); // now not retunr RPM
  }

  public double calculatePIDOutPutMotorA(double setPoint) {
    return pidControllerMotorA.calculate(getMotorARPM(MOTOR_A), setPoint);
  }

  public double calculatePIDOutPutMotorB(double setPoint) {
    return pidControllerMotorB.calculate(getMotorARPM(MOTOR_B), setPoint);
  }

  public boolean isAtSetPointMotorA(double waitTime) {
    return pidControllerMotorA.atSetpoint(waitTime);
  }

  public boolean isAtSetPointMotorB(double waitTime) {
    return pidControllerMotorB.atSetpoint(waitTime);
  }

  public void setSetPointMotorA(double setPoint) {
    pidControllerMotorA.setF(0); // TODO
    pidControllerMotorA.setSetpoint(setPoint);
  }

  public void setSetPointMotorB(double setPoint) {
    pidControllerMotorB.setF(0); // TODO
    pidControllerMotorB.setSetpoint(setPoint);
  }

  public double DistanceToRPM(double distance) {
    return 0; // TODO
  }

  @Override
  public void PrintValues() {
    linearShooterShuffleboard.addNum("MotorA RPM", getMotorARPM(MOTOR_A));
    linearShooterShuffleboard.addNum("MotorA PIDSetPoint", pidControllerMotorA.getSetpoint());
    linearShooterShuffleboard.addNum("MotorA PositionError", pidControllerMotorA.getPositionError());
    linearShooterShuffleboard.addBoolean("MotorA AtSetPoint", isAtSetPointMotorA(0.1));

    linearShooterShuffleboard.addNum("MotorB RPM", getMotorARPM(MOTOR_B));
    linearShooterShuffleboard.addNum("MotorB PIDSetPoint", pidControllerMotorB.getSetpoint());
    linearShooterShuffleboard.addNum("MotorB PositionError", pidControllerMotorB.getPositionError());
    linearShooterShuffleboard.addBoolean("MotorB AtSetPoint", isAtSetPointMotorB(0.1));

    linearShooterShuffleboard.addNum("getTransformMotorStatorCurrent", getTransformMotorStatorCurrent());
  }

  public static LinearShooter getinstance() {
    if (LinearShooter == null) {
      LinearShooter = new LinearShooter();
    }
    return LinearShooter;
  }
}
