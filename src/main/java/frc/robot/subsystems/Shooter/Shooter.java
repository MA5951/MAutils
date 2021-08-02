// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.Controlers.MAPidController;
import frc.robot.utils.Controlers.Interfaces.Controler;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Shooter extends MASubsystem {
  private MAMotorControler motorA;
  private MAMotorControler motorB;
  private static Shooter m_Shooter;
  private Controler controler;
  private MAShuffleboard shooterShuffleboard = new MAShuffleboard(ShooterConstants.KSUBSYSTEM_NAME);

  private Shooter() {
    motorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, ID5, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(motorA);
    motorB = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, ID6, false, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(motorB);
    motorB.follow(motorA);
    controler = new MAPidController(ShooterConstants.MOTOR_A_KP, ShooterConstants.MOTOR_A_KI,
        ShooterConstants.MOTOR_A_KD, 0, 70, -10, 10);
  }

  @Override
  public void periodic() {
    printValues();
  }

  /**
   * voltage -12 to 12, MotorA = 0
   */
  @Override
  public void setMotorPower(double power, int Index) {
    maMotorControlers.get(Index).setVoltage(power);
  }

  @Override
  public double getEncoderRPM() {

    return (maMotorControlers.get(ShooterConstants.MOTOR_A).getVelocity()
        + maMotorControlers.get(ShooterConstants.MOTOR_B).getVelocity()) / 2;
  }

  private double getAngularVelocity() {
    return getEncoderRPM() / 10;
  }

  public double distanceToRPM() {
    double LinearSpeed = Math.sqrt((Math.pow(getVxSpeed(), 2) + Math.pow(getVySpeed(), 2)));
    return MACalculations.fromLinearSpeedToRPM(LinearSpeed, ShooterConstants.KSHOOTER_GEAR);
  }

  private double getVxSpeed() {
    double d = limelight.distance();
    double head = Math.toDegrees(Math.tan(ShooterConstants.KSHOOT_ANGLE)) * d
        - (RobotConstants.KGRAVITY_ACCELERATION / 2) * Math.pow(d, 2);
    return Math.sqrt((head / ShooterConstants.KDELTA_Y));
  }

  private double getVySpeed() {
    double d = 3;// limelight.distance();
    double vx = getVxSpeed();
    return (((ShooterConstants.KDELTA_Y * vx) / d) - ((RobotConstants.KGRAVITY_ACCELERATION / 2) * d) / vx);
  }

  @Override
  public void setSetPoint(double setPoint) {

  }

  @Override
  public double calculatePIDOutput() {
    return controler.calculate(getEncoderRPM());
  }

  @Override
  public double getPositionError() {
    return controler.getPositionError();
  }

  @Override
  public double getSetpointPID() {
    return controler.getSetpoint();
  }

  @Override
  public boolean isPIDAtTarget() {
    return controler.atSetpoint();
  }

  @Override
  public void printValues() {
    shooterShuffleboard.addNum("MoonShooterAngularVelocity", getAngularVelocity());
    shooterShuffleboard.addNum("MoonShooterPIDSetPoint", getSetpointPID());
    shooterShuffleboard.addNum("MoonShooterPositionError", getPositionError());
    shooterShuffleboard.addNum("outPut", calculatePIDOutput());
    shooterShuffleboard.addBoolean("MoonShooterAtSetPoint", isPIDAtTarget());

  }

  public static Shooter getinstance() {
    if (m_Shooter == null) {
      m_Shooter = new Shooter();
    }
    return m_Shooter;
  }
}
