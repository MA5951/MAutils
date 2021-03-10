// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.subsystems.Automation.Automation;
import frc.robot.subsystems.Conveyor.Conveyor;
import frc.robot.subsystems.Conveyor.ConveyorConstants;
import frc.robot.subsystems.Shooter.MoonShooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.limelight;

public class Shooting extends CommandBase {
  /** Creates a new Shooting. */

  private Command visioCommand = new PIDVision(0);
  private double time = 0;

  public Shooting() {
    addRequirements(Automation.getinstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.camMode(0);
    time = 0;
    double setPoint = MoonShooter.getinstance().distanceToRPM();
    MoonShooter.getinstance().setSetPoint(setPoint);
    visioCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visioCommand.execute();
    MoonShooter.getinstance().setMotorVoltage(MoonShooter.getinstance().calculatePIDOutput(), ShooterConstants.MOTOR_A);
    if (visioCommand.isFinished() && MoonShooter.getinstance().isPIDAtTarget(0.1)) {

      if (Conveyor.getinstance().getStatorCurrent(ConveyorConstants.KCONVEYOR_MOTOR) < -13
          && Conveyor.getinstance().getStatorCurrent(ConveyorConstants.KTRANSPORTATION_MOTOR) < -37 && (time < 10)) {
        Automation.getinstance().conveyorControl(-1);
        time += RobotConstants.KDELTA_TIME;

      } else {
        Automation.getinstance().conveyorControl(1);
        time = 0;
      }

    } else {
      Automation.getinstance().conveyorControl(0);
      time = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visioCommand.end(true);
    MoonShooter.getinstance().setMotorVoltage(0, ShooterConstants.MOTOR_A);
    Automation.getinstance().conveyorControl(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
