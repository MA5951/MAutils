// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.controlers.MAPidController;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MoonShooter extends SubsystemBase implements MASubsystem {
  private MAMotorControler MotorA;
  private MAPidController PIDSpeedController;
  private static MoonShooter MoonShooter;
  private MAShuffleboard MoonShootersShuffleboard = new MAShuffleboard(ShooterConstants.SubsystemName);

  private MoonShooter() {
    MotorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(MotorA);

    PIDSpeedController = new MAPidController(ShooterConstants.MOTOR_A_KP, ShooterConstants.MOTOR_A_KI,
        ShooterConstants.MOTOR_A_KD, 0, 10, -12, 12);
  }

  @Override
  public void periodic() {
    PrintValues();
  }

  /**
   * voltage -12 to 12, MotorA = 0
   */
  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).setVoltage(power);
  }

  public double getMotorRPM() {
    return maMotorControlers.get(ShooterConstants.MOTOR_A).getVelocity();
  }

  public double calculatePIDOutPut(double setPoint) {
    return PIDSpeedController.calculate(getMotorRPM(), setPoint);
  }

  public boolean isAtSetPoint(double waitTime) {
    return PIDSpeedController.atSetpoint(waitTime);
  }

  public double getPositionError() {
    return PIDSpeedController.getPositionError();
  }

  public double getSetpoint() {
    return PIDSpeedController.getSetpoint();
  }

  public void setSetPoint(double setPoint) {
    PIDSpeedController.setF(0); // TODO
    PIDSpeedController.setSetpoint(setPoint);
  }

  public double distanceToRPM() {
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
    MoonShootersShuffleboard.addNum("RPM", getMotorRPM());
    MoonShootersShuffleboard.addNum("PIDSetPoint", getSetpoint());
    MoonShootersShuffleboard.addNum("PositionError", getPositionError());
    MoonShootersShuffleboard.addBoolean("AtSetPoint", isAtSetPoint(0.1));
  }

  public static MoonShooter getinstance() {
    if (MoonShooter == null) {
      MoonShooter = new MoonShooter();
    }
    return MoonShooter;
  }
}
