// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2.MACommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.MAUtils2.MASubsystem.PistonInterface;

public class MApistonCommand extends InstantCommand {
  private PistonInterface subsystem;
  private boolean value;

  public MApistonCommand(PistonInterface subsystem, boolean value) {
    this.value = value;
    this.subsystem = subsystem;
    addRequirements(this.subsystem.getInstance());
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
