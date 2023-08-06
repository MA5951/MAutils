// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultRunInternallyControlledSubsystem extends CommandBase {
  /** Creates a new DefultControlCommandInSubsystemControl. */
  private DefaultInternallyControlledSubsystem subsystem;
  private Supplier<Double> powerWhenCantMove;

  public DefaultRunInternallyControlledSubsystem(
    DefaultInternallyControlledSubsystem subsystem,
    Supplier<Double> powerWhenCantMove) {
      this.subsystem = subsystem;
      this.powerWhenCantMove = powerWhenCantMove;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subsystem.canMove()) {
      subsystem.calculate(subsystem.getSetPoint());
    } else {
      subsystem.setPower(powerWhenCantMove.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
