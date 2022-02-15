// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.commands.chassisCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.subsystem.chassis.BaseChassisPIDSubsystem;

public class ChassisPIDCommand extends CommandBase {
    /**
     * Creates a new ChassisPIDCommand.
     */
    private BaseChassisPIDSubsystem chassis;
    private boolean voltage;
    private double leftPower;
    private double rightPower;

    public ChassisPIDCommand(BaseChassisPIDSubsystem chassis, boolean voltage, double leftPower, double rightPower) {
        this.chassis = chassis;
        this.voltage = voltage;
        this.rightPower = rightPower;
        this.leftPower = leftPower;

        addRequirements(this.chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (voltage) {
            chassis.setRightVelocitySetpoint(rightPower * 12);
            chassis.setLeftVelocitySetpoint(leftPower * 12);
            chassis.setRightVoltage(chassis.getRightPID());
            chassis.setLeftVoltage(chassis.getLeftPID());
        } else {
            chassis.setRightVelocitySetpoint(rightPower);
            chassis.setLeftVelocitySetpoint(leftPower);
            chassis.setRightPercent(chassis.getRightPID());
            chassis.setLeftPercent(chassis.getLeftPID());
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.setRightPercent(0);
        chassis.setLeftPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
