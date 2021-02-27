/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Chassis.PIDVision;
import frc.robot.commands.Chassis.PIDVisionFeeder;
import frc.robot.utils.RobotConstants;

public class RobotContainer {

  public static XboxController OperatingJoystick = new XboxController(2);
  public static Joystick leftJoystick = new Joystick(0);
  public static Joystick rightJoystick = new Joystick(1);

  private JoystickButton leftJoystick5 = new JoystickButton(leftJoystick, 5);
  private JoystickButton rightJoystick5 = new JoystickButton(rightJoystick, 5);

  private JoystickButton leftJoystick3 = new JoystickButton(leftJoystick, 3);
  private JoystickButton rightJoystick3 = new JoystickButton(rightJoystick, 3);

  private JoystickButton AButton = new JoystickButton(OperatingJoystick, RobotConstants.A);
  private JoystickButton BButton = new JoystickButton(OperatingJoystick, RobotConstants.B);
  private JoystickButton YButton = new JoystickButton(OperatingJoystick, RobotConstants.Y);
  private JoystickButton XButton = new JoystickButton(OperatingJoystick, RobotConstants.X);
  private TriggerL triggerL = new TriggerL();
  private TriggerR triggerR = new TriggerR();
  private JoystickButton LB = new JoystickButton(OperatingJoystick, RobotConstants.LB);
  private JoystickButton RB = new JoystickButton(OperatingJoystick, RobotConstants.RB);
  private JoystickButton backkButton = new JoystickButton(OperatingJoystick, RobotConstants.BACK);
  private JoystickButton startButton = new JoystickButton(OperatingJoystick, RobotConstants.START);
  private JoystickButton stickLeft = new JoystickButton(OperatingJoystick, RobotConstants.STICK_LEFT);
  private JoystickButton stickRight = new JoystickButton(OperatingJoystick, RobotConstants.STICK_RIGHT);

  private POVButton POVUp = new POVButton(OperatingJoystick, RobotConstants.POV_UP);
  private POVButton POVDown = new POVButton(OperatingJoystick, RobotConstants.POV_DOWN);
  private POVButton POVLeft = new POVButton(OperatingJoystick, RobotConstants.POV_LEFT);
  private POVButton POVRight = new POVButton(OperatingJoystick, RobotConstants.POV_RIGHT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    triggerR.whileActiveOnce(new MAPath(0.1));

    leftJoystick5.whileHeld(new PIDVision(0));
    rightJoystick5.whileHeld(new PIDVision(0));

    leftJoystick3.whileHeld(new PIDVisionFeeder());
    rightJoystick3.whileHeld(new PIDVisionFeeder());
  }

 

}

class TriggerL extends Trigger {

  @Override
  public boolean get() {
    return RobotContainer.OperatingJoystick.getRawAxis(RobotConstants.L_TRIGER) > 0.5;
  }

}

class TriggerR extends Trigger {

  @Override
  public boolean get() {
    return RobotContainer.OperatingJoystick.getRawAxis(RobotConstants.R_TRIGER) > 0.5;
  }
}
