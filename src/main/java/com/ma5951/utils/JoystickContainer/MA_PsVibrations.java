package com.ma5951.utils.JoystickContainer;

import com.ma5951.utils.RobotConstants;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class MA_PsVibrations 
{
    public final PS4Controller controller;

    public MA_PsVibrations(int ID, int VibrationAmount,double VibrationStrenght, string VibrationType);
    {
        controller = new PS4Controller(ID);

        for (int i = 0; i <= VibrationAmount; i++)
        {
            controller.setRumble(RumbleType.kRightRumble, VibrationStrenght);
        }
    }
}

