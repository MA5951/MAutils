// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.ControlSubsystemInSubsystemControl;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlCommandInSubsystemControl extends CommandBase {
  /** Creates a new ControlCommand. */
  private ControlSubsystemInSubsystemControl subsystem;
  private Supplier<Double> setPoint;
  private boolean needToStopGivingPowerAtTheEnd;

  public ControlCommandInSubsystemControl(
    ControlSubsystemInSubsystemControl subsystem,
    Supplier<Double> setPoint,
    boolean needToStopGivingPowerAtTheEnd) {
    this.subsystem = subsystem;
    this.setPoint = setPoint;
    this.needToStopGivingPowerAtTheEnd = needToStopGivingPowerAtTheEnd;
    addRequirements(subsystem);
  }

  public ControlCommandInSubsystemControl(
    ControlSubsystemInSubsystemControl subsystem,
    double setPoint, boolean needToStopGivingPowerAtTheEnd) {
  this.subsystem = subsystem;
  this.setPoint = () -> setPoint;
  this.needToStopGivingPowerAtTheEnd = needToStopGivingPowerAtTheEnd;
  addRequirements(subsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subsystem.calculate(setPoint.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (needToStopGivingPowerAtTheEnd) {
      subsystem.setPower(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !subsystem.canMove() || subsystem.atPoint();
  }
}
