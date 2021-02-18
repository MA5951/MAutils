// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;

import frc.robot.utils.MASubsystem;
import frc.robot.utils.limelight;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MoonShooter extends SubsystemBase implements MASubsystem {
  private MAMotorControler MotorA;
  private MAPidController pidControllerMotorA;
  private static MoonShooter MoonShooter;
  private MAShuffleboard moonShootersShuffleboard = new MAShuffleboard(""); // TODO

  private MoonShooter() {
    MotorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolers(MotorA);
    // cahnge the tolorance
    pidControllerMotorA = new MAPidController(ShooterConstants.MOTOR_A_KP, ShooterConstants.MOTOR_A_KI,
        ShooterConstants.MOTOR_A_KD, 0, 10, -12, 12);
  }

  @Override
  public void periodic() {
    PrintValues();

  }

  private double getVxSpeed() {
    double d = limelight.getinstance().distance();
    double head = Math.tan(ShooterConstants.ShootAngle) * d - 5 * Math.pow(d, 2);
    return Math.sqrt((head / ShooterConstants.DeltaY));
  }

  private double getVySpeed() {
    return 0; 
      }

  /**
   * voltage -12 to 12, MotorA = 0
   */
  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).setvoltage(power);
  }

  public double getMotorARPM() {
    return maMotorControlers.get(ShooterConstants.MOTOR_A).getVelocity(); // now not retunr RPM
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

  public double DistanceToRPM() {
    return 0; // TODO
  }

  @Override
  public void PrintValues() {
    moonShootersShuffleboard.addNum("MotorA RPM", getMotorARPM());
    moonShootersShuffleboard.addNum("MotorA PIDSetPoint", pidControllerMotorA.getSetpoint());
    moonShootersShuffleboard.addNum("MotorA PositionError", pidControllerMotorA.getPositionError());
    moonShootersShuffleboard.addBoolean("MotorA AtSetPoint", isAtSetPointMotorA(0.1));
  }

  public static MoonShooter getinstance() {
    if (MoonShooter == null) {
      MoonShooter = new MoonShooter();
    }
    return MoonShooter;
  }
}
