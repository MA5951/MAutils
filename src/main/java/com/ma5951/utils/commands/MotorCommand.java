// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Generic motor command
 */

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.MotorSubsystem;

public class MotorCommand extends CommandBase {
  /** Creates a new MAMotorCommands. */
  private MotorSubsystem subsystem;
  private Supplier<Double> power;

  /**
   * Constructing a new instance of motor command class
   * @param subsystem Gets the subsystem that belongs to the motor for the command to run
   * @param power Gets the power value that the motor will operate at
   */

  public MotorCommand(MotorSubsystem subsystem, Supplier<Double> power) {
    this.subsystem = subsystem;
    this.power = power;
    addRequirements(subsystem);
  }

  /**
   * Constructing a new instance of motor command class
   * @param subsystem Gets the subsystem that belongs to the motor for the command to run
   * @param power Gets the power value that the motor will operate at
   */

  public MotorCommand(MotorSubsystem subsystem, double power) {
    this(subsystem, () -> power);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (subsystem.canMove())
      subsystem.setPower(power.get());
    else
      subsystem.setPower(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }
}
