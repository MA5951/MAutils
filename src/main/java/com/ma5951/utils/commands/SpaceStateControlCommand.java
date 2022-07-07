// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.ControlSubsystem;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SpaceStateControlCommand extends CommandBase {
  /** Creates a new SpaceStateControlCommand. */
  private LinearSystemLoop<N1, N1, N1> Loop;
  private Supplier<Double> setPoint;
  private ControlSubsystem subsystem;
  /**
   * @param systemIdentification1 
   * @param systemIdentification2
   * @param qelms Velocity error tolerance, in radians per second.
   *   Decrease this to more heavily penalize state excursion,
   *   or make the controller behave more
   *   aggressively.
   * @param relms Control effort (voltage) tolerance. Decrease this to more
   *  heavily penalize control effort, or make the controller less aggressive. 12 is a good
   *   starting point because that is the (approximate) maximum voltage of a battery.
   */
  public SpaceStateControlCommand(ControlSubsystem subsystem, double systemIdentification1,
   double systemIdentification2, Supplier<Double> setPoint, double modelAccuracy,
   double measurementAccuracy, double qelms, double relms) {
    // Use addRequirements() here to declare subsystem dependencies.
    final LinearSystem<N1, N1, N1> System =
    LinearSystemId.identifyVelocitySystem(systemIdentification1, systemIdentification2);
    final KalmanFilter<N1, N1, N1> Observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          System,
          VecBuilder.fill(modelAccuracy),
          VecBuilder.fill(measurementAccuracy),
          0.020);
    final LinearQuadraticRegulator<N1, N1, N1> Controller = 
    new LinearQuadraticRegulator<>(
      System,
      VecBuilder.fill(qelms),
      VecBuilder.fill(relms),
      0.020);
    final LinearSystemLoop<N1, N1, N1> loop = 
      new LinearSystemLoop<>(System, Controller, Observer, 12.0, 0.02);
    this.Loop = loop;
    this.setPoint = setPoint;
    this.subsystem = subsystem;
  }

  public SpaceStateControlCommand(ControlSubsystem subsystem, double systemIdentification1,
   double systemIdentification2, Supplier<Double> setPoint, double modelAccuracy,
   double measurementAccuracy, double qelms) {
      this (subsystem, systemIdentification1, systemIdentification2,
       setPoint, modelAccuracy, measurementAccuracy, qelms, 12);
    }

  public SpaceStateControlCommand(ControlSubsystem subsystem, double systemIdentification1,
   double systemIdentification2, double setPoint, double modelAccuracy,
   double measurementAccuracy, double qelms, double relms) {
    this(subsystem, systemIdentification1, systemIdentification2, () -> setPoint,
     modelAccuracy, measurementAccuracy, qelms, relms);
  }

  public SpaceStateControlCommand(ControlSubsystem subsystem, double systemIdentification1,
   double systemIdentification2, double setPoint, double modelAccuracy,
    double measurementAccuracy, double qelms) {
      this (subsystem, systemIdentification1, systemIdentification2,
       setPoint, modelAccuracy, measurementAccuracy, qelms, 12);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Loop.setNextR(VecBuilder.fill(setPoint.get()));
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Loop.correct(VecBuilder.fill(subsystem.getMeasurement()));
    Loop.predict(0.02);
    double nextVoltage = Loop.getU(0);
    subsystem.setVoltage(nextVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
