// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.utils.controlers.MAPidController;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class LinearShooter extends MASubsystem {
  private MAMotorControler motorA;
  private MAMotorControler motorB;
  private MAPidController pidSpeedController;
  private static LinearShooter m_Shooter;
  private MAShuffleboard shooterShuffleboard = new MAShuffleboard(ShooterConstants.KSUBSYSTEM_NAME);

  private LinearShooter() {
    motorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID10, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(motorA);
    pidSpeedController = new MAPidController(ShooterConstants.MOTOR_A_KP, ShooterConstants.MOTOR_A_KI,
        ShooterConstants.MOTOR_A_KD, 0, 10, -12, 12);

    motorB = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID11, false, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(motorB);

  }

  @Override
  public void periodic() {
    printValues();
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
    return pidSpeedController.calculate(getEncdoerRPM(ShooterConstants.MOTOR_A), setPoint);

  }

  @Override
  public boolean isPIDAtTarget(double waitTime) {
    return pidSpeedController.atSetpoint(waitTime);
  }

  @Override
  public double getPositionError() {
    return pidSpeedController.getPositionError();
  }

  @Override
  public double getSetpointPID() {
    return pidSpeedController.getSetpoint();
  }

  @Override
  public void setSetPoint(double setPoint) {
    pidSpeedController.setSetpoint(setPoint);
  }

  public double distanceToRPM(double distance) {
    double LinearSpeed = Math.sqrt((Math.pow(getVxSpeed(), 2) + Math.pow(getVySpeed(), 2)));
    return MACalculations.fromLinearSpeedToRPM(LinearSpeed, ShooterConstants.KSHOOTER_GEAR);
  }

  private double getVxSpeed() {
    double d = limelight.distance();
    double head = Math.tan(ShooterConstants.KSHOOT_ANGLE) * d
        - (RobotConstants.KGRAVITY_ACCELERATION / 2) * Math.pow(d, 2);
    return Math.sqrt((head / ShooterConstants.KDELTA_Y));
  }

  private double getVySpeed() {
    double d = limelight.distance();
    double vx = getVxSpeed();
    return (((ShooterConstants.KDELTA_Y * vx) / d) - ((RobotConstants.KGRAVITY_ACCELERATION / 2) * d) / vx);
  }

  @Override
  public void printValues() {
    shooterShuffleboard.addNum("MotorA RPM", getEncdoerRPM(ShooterConstants.MOTOR_A));
    shooterShuffleboard.addNum("LinearShooterPIDSetPoint", getSetpointPID());
    shooterShuffleboard.addNum("LinearShooterPositionError", getPositionError());
    shooterShuffleboard.addBoolean("LinearShooterAtSetPoint", isPIDAtTarget(0.1));
    shooterShuffleboard.addNum("MotorB RPM", getEncdoerRPM(ShooterConstants.MOTOR_B));
  }

  public static LinearShooter getinstance() {
    if (m_Shooter == null) {
      m_Shooter = new LinearShooter();
    }
    return m_Shooter;
  }
}
