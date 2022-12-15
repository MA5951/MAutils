package com.ma5951.utils.JoystickContainer;

import com.ma5951.utils.RobotConstants;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class MA_PsVibrations 
{
    public final PS4Controller controller;

    public MA_PsVibrations(int ControllerId, double VibrationAmount,double VibrationStrenght, int VibrationGapMillis, boolean IsRightVibration) throws InterruptedException
    {
        controller = new PS4Controller(ControllerId);
        
        new Thread(
            ()-> {
                try{
                    for (int i = 0; i <= VibrationAmount; i++)
                    {
                        if (IsRightVibration==true){
                            controller.setRumble(RumbleType.kRightRumble, VibrationStrenght);
                        }
                        else{
                            controller.setRumble(RumbleType.kLeftRumble, VibrationStrenght);
                        }
                        Thread.sleep(VibrationGapMillis);
                    }
                } catch (Exception e) {

                }
            }
        ).start();
        }
    }


