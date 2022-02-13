package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Chassis.Chassis;
import frc.robot.utils.RobotConstants;

public class ChasisPID extends CommandBase {

  public static Joystick leftJoystick = new Joystick(RobotConstants.KLEFT_JOYSTICK_PORT);
  public static Joystick rightJoystick = new Joystick(RobotConstants.KRIGHT_JOYSTICK_PORT);

  private Chassis chassis;
  public ChasisPID() {   
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.resetPID();
    
  }

  @Override
  public void execute() {
    System.out.println("swag");
    if (rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1)) {
      chassis.setRightVelocitySetpoint(-rightJoystick.getY() * 7 * 0.4);
      chassis.setLeftVelocitySetpoint(leftJoystick.getY() * 7 * 0.4);
    } 
    else {
      chassis.setRightVelocitySetpoint(-rightJoystick.getY() * 7);
      chassis.setLeftVelocitySetpoint(leftJoystick.getY() * 7);
    }
   
    chassis.setRightPercent(chassis.getRightPID(chassis.getRightEncoder())+chassis.getRightF());
    chassis.setLeftPercent(chassis.getLeftPID(chassis.getRightEncoder())+chassis.getLeftF());
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
