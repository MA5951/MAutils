// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Generic command that toogles pistons between open and close
 */

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ma5951.utils.subsystem.PistonSubsystem;

public class TogglePistonCommand extends CommandBase {
    private PistonSubsystem subsystem;

    /**
     * Constructing a new instance of piston command class
     * @param subsystem Gets the subsystem that belongs to the piston for the command to run
     */

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

    public void end(boolean interrupted) {
    }

    public boolean isFinished() {
        return true;
    }
}
