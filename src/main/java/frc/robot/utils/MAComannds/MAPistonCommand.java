// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAComannds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.MASubsystem.MASubsystem;

public class MAPistonCommand extends CommandBase {
  private MASubsystem currentSubsystem;
  private boolean value;

  public MAPistonCommand(MASubsystem currentSubsystem, boolean value) {
    this.value = value;
    this.currentSubsystem = currentSubsystem;
    addRequirements(this.currentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentSubsystem.setPiston(value);
  }

}
