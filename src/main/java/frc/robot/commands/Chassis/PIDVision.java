/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.limelight;

public class PIDVision extends CommandBase {
  private Chassis chassis;
  private double angle;

  public PIDVision(double angle) {
    this.angle = angle;
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.camMode(0);
    limelight.pipeline(0);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.pidVisionAngle(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.camMode(1);
    chassis.leftcontrol(0);
    chassis.rightcontrol(0);
    chassis.reset();
    chassis.setidilmodeBrake(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return chassis.isPIDVisionOnTargetAngle();
  }
}