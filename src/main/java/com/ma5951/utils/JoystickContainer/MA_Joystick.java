package com.ma5951.utils.JoystickContainer;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class MA_Joystick {
    public final Joystick joystick;

    public final JoystickButton joystick1Button;
    public final JoystickButton joystick2Button;
    public final JoystickButton joystick3Button;
    public final JoystickButton joystick4Button;
    public final JoystickButton joystick5Button;
    public final JoystickButton joystick6Button;
    public final JoystickButton joystick7Button;
    public final JoystickButton joystick8Button;
    public final JoystickButton joystick9Button;
    public final JoystickButton joystick10Button;
    public final JoystickButton joystick11Button;
    public final JoystickButton joystick12Button;
    
    public MA_Joystick(int ID) {
        joystick = new Joystick(ID);

        joystick1Button = new JoystickButton(joystick, 1);
        joystick2Button = new JoystickButton(joystick, 2);
        joystick3Button = new JoystickButton(joystick, 3);
        joystick4Button = new JoystickButton(joystick, 4);
        joystick5Button = new JoystickButton(joystick, 5);
        joystick6Button = new JoystickButton(joystick, 6);
        joystick7Button = new JoystickButton(joystick, 7);
        joystick8Button = new JoystickButton(joystick, 8);
        joystick9Button = new JoystickButton(joystick, 9);
        joystick10Button = new JoystickButton(joystick, 10);
        joystick11Button = new JoystickButton(joystick, 11);
        joystick12Button = new JoystickButton(joystick, 12);
    }
}
