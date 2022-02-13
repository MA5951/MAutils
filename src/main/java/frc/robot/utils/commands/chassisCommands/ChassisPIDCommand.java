// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.commands.chassisCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.subsystem.chassis.BasicChassisPIDSubsystem;
public class ChassisPIDCommand extends CommandBase {
  /** Creates a new ChassisPIDCommand. */
  private BasicChassisPIDSubsystem chassis;
  private boolean voltage;
  private double LeftPower;
  private double RightPower;

  public ChassisPIDCommand(BasicChassisPIDSubsystem chassis, boolean voltage, double LeftPower, double RightPower) {
    this.chassis = chassis;
    this.voltage = voltage;
    this.RightPower = RightPower;
    this.LeftPower = LeftPower;
    addRequirements(this.chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (voltage) {
      chassis.setRightVelocitySetpoint(RightPower * 12);
      chassis.setLeftVelocitySetpoint(LeftPower * 12);
      chassis.setRightVoltage(chassis.rightVelocityMApathPIDOutput());
      chassis.setLeftVoltage(chassis.leftVelocityMApathPIDOutput());
    } else {
      chassis.setRightVelocitySetpoint(RightPower);
      chassis.setLeftVelocitySetpoint(LeftPower);
      chassis.setRightPercent(chassis.rightVelocityMApathPIDOutput());
      chassis.setLeftPercent(chassis.rightVelocityMApathPIDOutput());
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
