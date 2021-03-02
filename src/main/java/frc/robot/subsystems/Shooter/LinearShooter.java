// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class LinearShooter extends MASubsystem {
  private MAMotorControler MotorA;
  private MAMotorControler MotorB;
  private MAPidController PIDSpeedController;
  private static LinearShooter LinearShooter;
  private MAShuffleboard LinearShooterShuffleboard = new MAShuffleboard(ShooterConstants.SubsystemName);

  private LinearShooter() {
    MotorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(MotorA);
    PIDSpeedController = new MAPidController(ShooterConstants.MOTOR_A_KP, ShooterConstants.MOTOR_A_KI,
        ShooterConstants.MOTOR_A_KD, 0, 10, -12, 12);

    MotorB = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID2, false, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(MotorB);

  }

  @Override
  public void periodic() {
    PrintValues();
  }

  /**
   * voltage -12 to 12, MotorA = 0 , MotorB = 1
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).setVoltage(power);
  }

  @Override
  public double getEncdoerRPM(int indax) {
    return maMotorControlers.get(indax).getVelocity();
  }

  @Override
  public double calculatePIDOutput(double setPoint) {
    return PIDSpeedController.calculate(getEncdoerRPM(ShooterConstants.MOTOR_A), setPoint);

  }

  @Override
  public boolean isPIDAtTarget(double waitTime) {
    return PIDSpeedController.atSetpoint(waitTime);
  }

  @Override
  public double getPositionError() {
    return PIDSpeedController.getPositionError();
  }

  @Override
  public double getSetpointPID() {
    return PIDSpeedController.getSetpoint();
  }

  @Override
  public void setSetPoint(double setPoint) {
    PIDSpeedController.setSetpoint(setPoint);
  }

  public double DistanceToRPM(double distance) {
    double LinearSpeed = Math.sqrt((Math.pow(getVxSpeed(), 2) + Math.pow(getVySpeed(), 2)));
    return MACalculations.FromLinearSpeedToRPM(LinearSpeed, ShooterConstants.ShooterGear);
  }

  private double getVxSpeed() {
    double d = limelight.distance();
    double head = Math.tan(ShooterConstants.ShootAngle) * d
        - (RobotConstants.KGRAVITY_ACCELERATION / 2) * Math.pow(d, 2);
    return Math.sqrt((head / ShooterConstants.DeltaY));
  }

  private double getVySpeed() {
    double d = limelight.distance();
    double vx = getVxSpeed();
    return (((ShooterConstants.DeltaY * vx) / d) - ((RobotConstants.KGRAVITY_ACCELERATION / 2) * d) / vx);
  }

  @Override
  public void PrintValues() {
    LinearShooterShuffleboard.addNum("MotorA RPM", getEncdoerRPM(ShooterConstants.MOTOR_A));
    LinearShooterShuffleboard.addNum("PIDSetPoint", getSetpointPID());
    LinearShooterShuffleboard.addNum("PositionError", getPositionError());
    LinearShooterShuffleboard.addBoolean("AtSetPoint", isPIDAtTarget(0.1));
    LinearShooterShuffleboard.addNum("MotorB RPM", getEncdoerRPM(ShooterConstants.MOTOR_B));
  }

  public static LinearShooter getinstance() {
    if (LinearShooter == null) {
      LinearShooter = new LinearShooter();
    }
    return LinearShooter;
  }
}
