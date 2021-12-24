// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2.MACommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.MAUtils2.MASubsystem.MotorInterface;
import frc.robot.MAUtils2.MASubsystem.ControlInterface;

public class MAControlCommand<MAMotorInterface extends MotorInterface & ControlInterface & Subsystem> extends CommandBase {
  /** Creates a new MAControlCommand. */

  private MAMotorInterface subSystem;
  private double SetPoint;
  private boolean stoppable;
  private boolean voltege;


  public MAControlCommand(MAMotorInterface subSystem, double setPoint, boolean stoppable, boolean voltege) {
    this.subSystem = subSystem;
    this.SetPoint = setPoint;
    this.stoppable = stoppable;
    this.voltege = voltege;

    addRequirements(subSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (voltege) {
      subSystem.setVoltege(subSystem.calculate(SetPoint));
    } else {
      subSystem.setPower(subSystem.calculate(SetPoint));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subSystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stoppable) {
      if (subSystem.atSetpoint()) {
        return true;
      }
    }
    return false;
  }
}
