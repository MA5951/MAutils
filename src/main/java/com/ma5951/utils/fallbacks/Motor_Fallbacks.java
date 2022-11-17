package com.ma5951.utils.fallbacks;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

public class Motor_Fallbacks {


    double busVoltage;
    double position;
    double lastPosition;
    double voltageThreshold;
    double voltageInput;
    double movesThreshold;
    boolean isEnviromentDependent;
    // List<String> motor_types= Arrays.AsList("Falcon","SparkMax","Neo","Word4");

    public boolean MaFaults(IMotorController motor)
    {
        TalonFX talon;

        
        motor.getMotorOutputVoltage();
        motor.enableVoltageCompensation(true);
        motor.getTemperature();
        motor.getFaults();

        boolean underVol = motor.getFaults().underVoltage();
        motor.getLastError();
        //command to see if a motor is connected to the robot
        
    } 

    public Motor_Fallbacks(double voltageInput, double position,
                           double lastPosition, double VoltageUsed, 
                           double movesThreshold, boolean isEnviromentDependent, 
                           String moudle_used, Object motor)
    {
        // moudle_used = moudle_used.toLowerCase();
        // if (moudle_used == "falcon"){

        // }

        //
        this.voltageInput=voltageInput;
        this.position=position;
        this.lastPosition=lastPosition;
        this.voltageThreshold=voltageThreshold;
        this.busVoltage=busVoltage;
        this.movesThreshold=movesThreshold;
        this.isEnviromentDependent=isEnviromentDependent;
    }

    public Motor_Fallbacks(double voltage, double VoltageUsed)
    {
        this.voltage=voltage;
        this.voltageUsed=voltageUsed;
    }


    public static boolean allMotorChecks(double voltage, double position,
                                         double lastPosition, double voltageThreshold, 
                                         double VoltageUsed, double movesThreshold,
                                         boolean isEnviromentDependent)
    {
        return  voltageCheck(voltage, VoltageUsed, voltageThreshold) || 
                encoderDriftCheck(position, lastPosition, movesThreshold) || 
                movesCheck(voltage, position, lastPosition, movesThreshold ) ||
                movesWhenSouldent(voltage, position, lastPosition, movesThreshold, isEnviromentDependent);

                
    }

    public static boolean voltageCheck(double voltage, double VoltageUsed)
    {
        return voltage - 2 >= VoltageUsed && voltage + 2 <= VoltageUsed;
        //add controller rumble if false and send to shuffleboard
    }

    public static boolean voltageCheck(double voltage, double VoltageUsed, double threshhold)
    {
        return voltage - threshhold >= VoltageUsed && voltage + threshhold <= VoltageUsed;
        //add controller rumble if false and send to shuffleboard
    }
    

    public static boolean encoderDriftCheck(double position, double lastPosition, double threshold)
    {
        return position + threshold <= lastPosition && position - threshold >= lastPosition;
        //add controller rumble if false and send to shuffleboard
        //add timer so there wont be jumps back and forth
    }


    public static boolean movesCheck(double voltage, double position, double lastPosition, double threshold)
    {
        return voltage > 0 && position - threshold >= lastPosition && position + threshold <= lastPosition;
    }

    public static boolean movesWhenSouldent(double givenVoltage, double position, double lastPosition, double threshold, boolean type)
    {
        if (type = false)
        {
            return givenVoltage == 0 && position - threshold >= lastPosition && position + threshold <= lastPosition;
        }
        else
        {
            return false;
        }
    }
}
