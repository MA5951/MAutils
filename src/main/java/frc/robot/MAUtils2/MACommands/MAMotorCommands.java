// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2.MACommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.MAUtils2.MASubsystem.MotorInterface;

public class MAMotorCommands <MAMotor extends MotorInterface & Subsystem> extends InstantCommand  {
  /** Creates a new MAMotorCommands. */
  private MAMotor subsystem;
  private double power;

  public MAMotorCommands(MAMotor subsystem, double power) {
    this.subsystem = subsystem;
    this.power = power;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    subsystem.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }
}
