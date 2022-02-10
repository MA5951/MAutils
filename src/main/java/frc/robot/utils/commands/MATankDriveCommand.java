package frc.robot.utils.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.JoystickContainer;
import frc.robot.utils.subsystem.ChassisInterfaceSubsystem;
import frc.robot.utils.subsystem.MotorInterfaceSubsystem;

public class MATankDriveCommand extends CommandBase {
  private ChassisInterfaceSubsystem chassis;
  private Joystick leftJoystick;
  private Joystick rightJoystick;

  public MATankDriveCommand(Joystick leftJoystick,Joystick rightJoystick,ChassisInterfaceSubsystem chassis) {
    this.leftJoystick=leftJoystick;
    this.rightJoystick=rightJoystick;
    this.chassis=chassis;
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (rightJoystick.getY() > ChassisConstants.KTHRESHOLD
        || rightJoystick.getY() < -ChassisConstants.KTHRESHOLD) {
      if (rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1)) {

        chassis.rightcontrol(rightJoystick.getY() * ChassisConstants.KSCALE);
      } else {
        chassis.rightcontrol(rightJoystick.getY());
      }
    } else {

      chassis.rightcontrol(0);
    }

    if (leftJoystick.getY() > ChassisConstants.KTHRESHOLD
        || leftJoystick.getY() < -ChassisConstants.KTHRESHOLD) {

      if (rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1)) {

        chassis.leftcontrol(leftJoystick.getY() * ChassisConstants.KSCALE);
      } else {
        chassis.leftcontrol(leftJoystick.getY());
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
