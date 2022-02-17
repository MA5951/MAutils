// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class PistonCommand extends InstantCommand {
  private PistonSubsystem subsystem;
  private boolean value;

  public PistonCommand(PistonSubsystem subsystem, boolean value) {
    this.value = value;
    this.subsystem = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (value){
      subsystem.open();
    }
    else{
      subsystem.close();
    }
  }
}
