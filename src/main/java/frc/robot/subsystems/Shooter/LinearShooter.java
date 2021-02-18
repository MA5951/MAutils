// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MAPidController;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class LinearShooter extends SubsystemBase implements MASubsystem {
  private MAMotorControler MotorA;
  private MAMotorControler MotorB;
  private MAPidController pidControllerMotorA;

  private static LinearShooter LinearShooter;
  private MAShuffleboard linearShooterShuffleboard = new MAShuffleboard(ShooterConstants.SubsystemName); // TODO

  private LinearShooter() {
    MotorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID1, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolers(MotorA);
    pidControllerMotorA = new MAPidController(ShooterConstants.MOTOR_A_KP, ShooterConstants.MOTOR_A_KI,
        ShooterConstants.MOTOR_A_KD, 0, 10, -12, 12); // cahnge the

    MotorB = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID2, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolers(MotorB);

  }

  @Override
  public void periodic() {
    PrintValues();
  }

  /**
   * voltage -12 to 12, MotorA = 0 , MotorB = 1 
   */
  @Override
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).setvoltage(power);
  }

  public double getMotorARPM(int Indax) {
    return maMotorControlers.get(Indax).getVelocity(); // now not retunr RPM
  }

  public double calculatePIDOutPutMotorA(double setPoint) {
    return pidControllerMotorA.calculate(getMotorARPM(ShooterConstants.MOTOR_A), setPoint);
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
    linearShooterShuffleboard.addNum("MotorA RPM", getMotorARPM(ShooterConstants.MOTOR_A));
    linearShooterShuffleboard.addNum("MotorA PIDSetPoint", pidControllerMotorA.getSetpoint());
    linearShooterShuffleboard.addNum("MotorA PositionError", pidControllerMotorA.getPositionError());
    linearShooterShuffleboard.addBoolean("MotorA AtSetPoint", isAtSetPointMotorA(0.1));
    linearShooterShuffleboard.addNum("MotorB RPM", getMotorARPM(ShooterConstants.MOTOR_B));
  }

  public static LinearShooter getinstance() {
    if (LinearShooter == null) {
      LinearShooter = new LinearShooter();
    }
    return LinearShooter;
  }
}
