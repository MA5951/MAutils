// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.commands.chassisCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.JoystickContainer;
import frc.robot.utils.subsystem.chassis.BasicChassisSubsystem;

public class BasicChassisCommand extends CommandBase {
  /** Creates a new BasicChassisCommand. */
  private BasicChassisSubsystem chassis;
  private double LeftPower;
  private double RightPower;

  public BasicChassisCommand(BasicChassisSubsystem chassis, double LeftPower, double RightPower) {
    this.chassis = chassis;
    this.LeftPower = LeftPower;
    this.RightPower = RightPower;
    addRequirements(this.chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setLeftPercent(LeftPower);
    chassis.setRightPercent(RightPower);
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
