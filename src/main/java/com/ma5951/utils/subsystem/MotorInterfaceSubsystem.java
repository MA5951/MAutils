package com.ma5951.utils.subsystem;

import com.ma5951.utils.RobotConstants;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface MotorInterfaceSubsystem extends Subsystem {
    public void setVoltage (double voltege);
    
    default void setPower(double power){
        setVoltage(power * RobotConstants.voltage);
    }

    public double getVoltage();

    default double getPower(){
        return getVoltage() * RobotConstants.voltage;
    }
}