/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Chassis.Chassis;
import frc.robot.Chassis.ChassisConstants;
import frc.robot.utils.JoystickContainer;

public class TankDrive extends CommandBase {
  private Chassis chassis;

  public TankDrive() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (JoystickContainer.rightJoystick.getY() > ChassisConstants.KTHRESHOLD
        || JoystickContainer.rightJoystick.getY() < -ChassisConstants.KTHRESHOLD) {
      if (JoystickContainer.rightJoystick.getRawButton(1) || JoystickContainer.leftJoystick.getRawButton(1)) {

        chassis.setRightPercent(JoystickContainer.rightJoystick.getY() * ChassisConstants.KSCALE);
      } else {
        chassis.setRightPercent(JoystickContainer.rightJoystick.getY());
      }
    } else {

      chassis.setRightVoltage(0);
    }

    if (JoystickContainer.leftJoystick.getY() > ChassisConstants.KTHRESHOLD
        || JoystickContainer.leftJoystick.getY() < -ChassisConstants.KTHRESHOLD) {

      if (JoystickContainer.rightJoystick.getRawButton(1) || JoystickContainer.leftJoystick.getRawButton(1)) {

        chassis.setLeftPercent(JoystickContainer.leftJoystick.getY() * ChassisConstants.KSCALE);
      } else {
        chassis.setLeftPercent(JoystickContainer.leftJoystick.getY());
      }
    } else {

      chassis.setLeftVoltage(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.setLeftVoltage(0);
    chassis.setRightVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}