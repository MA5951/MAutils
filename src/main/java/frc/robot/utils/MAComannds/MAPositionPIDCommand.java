// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAComannds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.MASubsystem.MASubsystem;

public class MAPositionPIDCommand extends CommandBase {
  private MASubsystem currentSubsystem;
  private double setPoint;
  private int indax;

  public MAPositionPIDCommand(MASubsystem currentSubsystem, double setPoint, int indax) {
    this.setPoint = setPoint;
    this.currentSubsystem = currentSubsystem;
    this.indax = indax;
    addRequirements(this.currentSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = this.currentSubsystem.calculatePIDOutput(setPoint);
    this.currentSubsystem.setMotorPower(power, indax);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.currentSubsystem.setMotorPower(0, indax);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.currentSubsystem.isPIDAtTarget(0.1);
  }
}
