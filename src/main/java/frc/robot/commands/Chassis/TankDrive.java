/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.ButtonContainer;
import frc.robot.subsystems.Chassis.Chassis;

public class TankDrive extends CommandBase {
  private Chassis chassis;

  public TankDrive() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.rampRate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (ButtonContainer.rightJoystick.getY() > 0.1 || ButtonContainer.rightJoystick.getY() < -0.1) {
      if (ButtonContainer.rightJoystick.getRawButton(1) || ButtonContainer.leftJoystick.getRawButton(1)) {

        chassis.rightcontrol(ButtonContainer.rightJoystick.getY() * 0.3);
      } else {
        chassis.rightcontrol(ButtonContainer.rightJoystick.getY());
      }
    } else {

      chassis.rightcontrol(0);
    }

    if (ButtonContainer.leftJoystick.getY() > 0.1 || ButtonContainer.leftJoystick.getY() < -0.1) {

      if (ButtonContainer.rightJoystick.getRawButton(1) || ButtonContainer.leftJoystick.getRawButton(1)) {

        chassis.leftcontrol(ButtonContainer.leftJoystick.getY() * 0.3);
      } else {
        chassis.leftcontrol(ButtonContainer.leftJoystick.getY());
      }
    } else {

      chassis.leftcontrol(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.leftcontrol(0);
    chassis.rightcontrol(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}