// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package utils.commands.chassisCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import utils.subsystem.chassis.BaseChassisSubsystem;

public class BasicChassisCommand extends CommandBase {
    /**
     * Creates a new BasicChassisCommand.
     */
    private BaseChassisSubsystem chassis;
    private double leftPower;
    private double rightPower;

    public BasicChassisCommand(BaseChassisSubsystem chassis, double leftPower, double rightPower) {
        this.chassis = chassis;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        addRequirements(this.chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        chassis.setLeftPercent(leftPower);
        chassis.setRightPercent(rightPower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.setLeftPercent(0);
        chassis.setRightPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
