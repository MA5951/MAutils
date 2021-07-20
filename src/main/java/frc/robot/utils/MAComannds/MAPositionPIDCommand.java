// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAComannds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.MASubsystem.MASubsystem;

public class MAPositionPIDCommand extends CommandBase {
  private MASubsystem currentSubsystem;
  private double setPoint;
  private int index;

  public MAPositionPIDCommand(MASubsystem currentSubsystem, double setPoint, int index) {
    this.setPoint = setPoint;
    this.currentSubsystem = currentSubsystem;
    this.index = index;
    addRequirements(currentSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentSubsystem.setSetPoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = this.currentSubsystem.calculatePIDOutput(setPoint);
    currentSubsystem.setMotorVoltage(power, index);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentSubsystem.setMotorPower(0, index);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.currentSubsystem.isPIDAtTarget();
  }
}
