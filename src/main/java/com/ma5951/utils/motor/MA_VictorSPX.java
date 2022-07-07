// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class MA_VictorSPX implements MotorController {

    private WPI_VictorSPX victorSPX;

    /**
     * Constructing a new instance of VictorSPX class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param rampRate Gets the ramp rate to set to the motor
     * @param mode Gets the mode of the motor - brake or coast
    */

    public MA_VictorSPX(int id, boolean inverted, double rampRate, boolean mode) {
        victorSPX = new WPI_VictorSPX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mode);
    }


    /**
     * Constructing a new instance of VictorSPX class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param mode Gets the mode of the motor - brake or coast
    */

    public MA_VictorSPX(int id, boolean inverted, boolean mode) {
        victorSPX = new WPI_VictorSPX(id);
        setInverted(inverted);
        changeMode(mode);
    }

    /**
     * Sets the motor voltage
     * @param voltage Gets the voltage that the motor will operate at
    */

    @Override
    public void setVoltage(double voltage) {
        victorSPX.setVoltage(voltage);
    }


    /**
     * Sets the motor power
     * @param power Gets the power value that the motor will operate at
     */
    @Override
    public void setPower(double power) {
        victorSPX.set(ControlMode.PercentOutput, power);
    }

    /**
     * Gets the output of the motor
     * @return The current output of the motor
     */

    @Override
    public double getOutput() {
        return victorSPX.get();
    }
    

    /**
     * Sets the ramp rate
     * @param rampRate Gets the ramp rate to be set
     */

    @Override
    public void configRampRate(double rampRate) {
        victorSPX.configOpenloopRamp(rampRate);
        victorSPX.configClosedloopRamp(rampRate);

    }


    /**
     * Sets the motor to be invert or not
     * @param setInverted Gets if sets the motor to be invert or not
     */
    @Override
    public void setInverted(Boolean setInverted) {
        victorSPX.setInverted(setInverted);
    }


    /**
     * @param changeMode Changes the mode of the motor - brake or coast
     * @param onOff Gets the mode of the motor to be set
     */
    @Override
    public void changeMode(boolean onOff) {
        victorSPX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /**
     * @param getVictorSPX returns the motor controller objeect
     */
    private WPI_VictorSPX getVictorSPX() { return victorSPX; }


    /**
     * @param follow set motor to follow
     * @param motor the motor you want to follow at
     */
    public void follow(MA_VictorSPX motor) {
        victorSPX.follow(motor.getVictorSPX());
    }

    /**
     * @param getID returns the motor ID
     */
    @Override
    public int getID() {
        return victorSPX.getDeviceID();
    }

}