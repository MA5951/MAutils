// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.MASubsystem.SubConstants;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class MoonShooter extends MASubsystem {
  private MAMotorControler motorA;
  private MAMotorControler motorB;
  private static MoonShooter m_Shooter;
  private LinearSystem<N1, N1, N1> flyWheelLinearSystem;
  private KalmanFilter<N1, N1, N1> flyWheelKalmanFilter;
  private LinearQuadraticRegulator<N1, N1, N1> flyWheelLinearQuadraticRegulator;
  private LinearSystemLoop<N1, N1, N1> flyWhLinearSystemLoop;
  private MAShuffleboard shooterShuffleboard = new MAShuffleboard(ShooterConstants.KSUBSYSTEM_NAME);

  private MoonShooter() {
    motorA = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID5, true, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(motorA);
    motorB = new MAMotorControler(MOTOR_CONTROLL.SPARKMAXBrushless, IDMotor.ID6, false, 0, false, ENCODER.Encoder);
    setMAMotorComtrolersList(motorB);
    motorB.follow(motorA);

    flyWheelLinearSystem = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2),
        ShooterConstants.kFlywheelMomentOfInertia, ShooterConstants.KSHOOTER_GEAR);

    flyWheelKalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), flyWheelLinearSystem, VecBuilder.fill(3),
        VecBuilder.fill(0.01), RobotConstants.KDELTA_TIME);

    flyWheelLinearQuadraticRegulator = new LinearQuadraticRegulator<>(flyWheelLinearSystem, VecBuilder.fill(7.0),
        VecBuilder.fill(12), RobotConstants.KDELTA_TIME);

    flyWhLinearSystemLoop = new LinearSystemLoop<>(flyWheelLinearSystem, flyWheelLinearQuadraticRegulator,
        flyWheelKalmanFilter, 10, RobotConstants.KDELTA_TIME);
  }

  @Override
  public void periodic() {
    printValues();
  }

  /**
   * voltage -12 to 12, MotorA = 0
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).setVoltage(power);
  }

  @Override
  public double getEncdoerRPM() {

    return (maMotorControlers.get(ShooterConstants.MOTOR_A).getVelocity()
        + maMotorControlers.get(ShooterConstants.MOTOR_B).getVelocity()) / 2;
  }

  private double getAngularVelocity() {
    return getEncdoerRPM() / 10;
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
    flyWhLinearSystemLoop.setNextR(VecBuilder.fill(setPoint / 10)); // in radain per
    // secend
  }

  @Override
  public double calculatePIDOutput() {

    flyWhLinearSystemLoop.correct(VecBuilder.fill(getAngularVelocity()));
    flyWhLinearSystemLoop.predict(RobotConstants.KDELTA_TIME);
    return flyWhLinearSystemLoop.getU(0);

  }

  @Override
  public double getPositionError() {
    return flyWhLinearSystemLoop.getError(0); // TODO the right indax
  }

  @Override
  public double getSetpointPID() {
    return flyWhLinearSystemLoop.getNextR(0); // TODO the right row
  }

  @Override
  public boolean isPIDAtTarget() {
    return Math.abs(getPositionError()) < 70;
  }

  @Override
  public void printValues() {
    shooterShuffleboard.addNum("MoonShooterAngularVelocity", getAngularVelocity());
    shooterShuffleboard.addNum("MoonShooterPIDSetPoint", getSetpointPID());
    shooterShuffleboard.addNum("MoonShooterPositionError", getPositionError());
    shooterShuffleboard.addNum("outPut", calculatePIDOutput());
    shooterShuffleboard.addBoolean("MoonShooterAtSetPoint", isPIDAtTarget());

  }

 

  public static MoonShooter getinstance() {
    if (m_Shooter == null) {
      m_Shooter = new MoonShooter();
    }
    return m_Shooter;
  }
}
