// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAComannds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.MASubsystem.MASubsystem;
import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

public class MAVelocityPIDCommand extends CommandBase {
  private MASubsystem currentSubsystem;
  private Runnable setPoint;
  private int indax;

  public MAVelocityPIDCommand(MASubsystem currentSubsystem, Runnable setPoint, int indax) {
    this.currentSubsystem = currentSubsystem;
    this.setPoint = requireNonNullParam(setPoint, "setPoint", "MAVelocityPIDCommand");
    this.indax = indax;
    addRequirements(currentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint.run();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double voltage = currentSubsystem.calculatePIDOutput();
    currentSubsystem.setMotorVoltage(voltage, indax);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentSubsystem.setMotorVoltage(0, indax);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
