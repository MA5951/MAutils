/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickContainer {

  public static XboxController OperatingJoystick = new XboxController(RobotConstants.KOPERATING_JOYSTICK_PORT);
  public static Joystick leftJoystick = new Joystick(RobotConstants.KLEFT_JOYSTICK_PORT);
  public static Joystick rightJoystick = new Joystick(RobotConstants.KRIGHT_JOYSTICK_PORT);

  public static JoystickButton leftJoystick5 = new JoystickButton(leftJoystick, 5);
  public static JoystickButton rightJoystick5 = new JoystickButton(rightJoystick, 5);

  public static JoystickButton leftJoystick3 = new JoystickButton(leftJoystick, 3);
  public static JoystickButton rightJoystick3 = new JoystickButton(rightJoystick, 3);

  public static JoystickButton AButton = new JoystickButton(OperatingJoystick, RobotConstants.A);
  public static JoystickButton BButton = new JoystickButton(OperatingJoystick, RobotConstants.B);
  public static JoystickButton YButton = new JoystickButton(OperatingJoystick, RobotConstants.Y);
  public static JoystickButton XButton = new JoystickButton(OperatingJoystick, RobotConstants.X);
  public static TriggerL triggerL = new TriggerL();
  public static TriggerR triggerR = new TriggerR();
  public static JoystickButton LB = new JoystickButton(OperatingJoystick, RobotConstants.LB);
  public static JoystickButton RB = new JoystickButton(OperatingJoystick, RobotConstants.RB);
  public static JoystickButton backkButton = new JoystickButton(OperatingJoystick, RobotConstants.BACK);
  public static JoystickButton startButton = new JoystickButton(OperatingJoystick, RobotConstants.START);
  public static JoystickButton stickLeft = new JoystickButton(OperatingJoystick, RobotConstants.STICK_LEFT);
  public static JoystickButton stickRight = new JoystickButton(OperatingJoystick, RobotConstants.STICK_RIGHT);

  public static POVButton POVUp = new POVButton(OperatingJoystick, RobotConstants.POV_UP);
  public static POVButton POVDown = new POVButton(OperatingJoystick, RobotConstants.POV_DOWN);
  public static POVButton POVLeft = new POVButton(OperatingJoystick, RobotConstants.POV_LEFT);
  public static POVButton POVRight = new POVButton(OperatingJoystick, RobotConstants.POV_RIGHT);

}

class TriggerL extends Trigger {

  @Override
  public boolean get() {
    return JoystickContainer.OperatingJoystick.getRawAxis(RobotConstants.L_TRIGER) > 0.5;
  }

}

class TriggerR extends Trigger {

  @Override
  public boolean get() {
    return JoystickContainer.OperatingJoystick.getRawAxis(RobotConstants.R_TRIGER) > 0.5;
  }
}
