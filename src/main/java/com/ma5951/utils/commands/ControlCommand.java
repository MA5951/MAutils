// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.ma5951.utils.subsystem.ControlSubsystem;

public class ControlCommand extends CommandBase {
  /** Creates a new MAControlCommand. */

  private ControlSubsystem subsystem;
  private Supplier<Double> setpoint;
  private boolean stoppable;
  private boolean voltage;

  public ControlCommand(ControlSubsystem subsystem, Supplier<Double> setpoint, boolean stoppable, boolean voltage) {
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.stoppable = stoppable;
    this.voltage = voltage;

    addRequirements(subsystem);
  }

  public ControlCommand(ControlSubsystem subsystem, double setpoint, boolean stoppable, boolean voltage) {
    this(subsystem, () -> setpoint, stoppable, voltage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subsystem.canMove()) {
      if (voltage) {
        subsystem.setVoltage(subsystem.calculate(setpoint.get()));
      } else {
        subsystem.setPower(subsystem.calculate(setpoint.get()));
      }
    } else {
      subsystem.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stoppable) {
      if (subsystem.atSetpoint()) {
        return true;
      }
    }
    return false;
  }
}
