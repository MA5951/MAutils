// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Sets piston to desire state - open or close 
 */

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.PistonSubsystem;

public class PistonCommand extends CommandBase {
  private PistonSubsystem subsystem;
  private Supplier<Boolean> value;

  /**
   * Constructing a new instance of piston command class
   * @param subsystem Gets the subsystem that belongs to the piston for the command to run
   * @param value Gets the value that the piston will set to
   */

  public PistonCommand(PistonSubsystem subsystem, Supplier<Boolean> value) {
    this.value = value;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  /**
   * Constructing a new instance of piston command class
   * @param subsystem Gets the subsystem that belongs to the piston for the command to run
   * @param value Gets the value that the piston will set to
   */

  public PistonCommand(PistonSubsystem subsystem, boolean value) {
    this(subsystem, () -> value);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (value.get()) {
      subsystem.open();
    } else {
      subsystem.close();
    }
  }

  public void end(boolean interrupted) {
  }

  public boolean isFinished() {
    return true;
  }
}
