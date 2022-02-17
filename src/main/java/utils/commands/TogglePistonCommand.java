// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package utils.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import utils.subsystem.PistonSubsystem;

public class TogglePistonCommand extends InstantCommand {
    private PistonSubsystem subsystem;

    public TogglePistonCommand(PistonSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (subsystem.isOpen()) {
            subsystem.close();
        } else {
            subsystem.open();
        }
    }
}
