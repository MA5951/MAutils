// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.motor;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MA_TalonSRX implements MotorController, MotorSensors {
    private WPI_TalonSRX talonSRX;


    /**
     * Constructing a new instance of TalonSRX class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param rampRate Gets the ramp rate to set to the motor
     * @param mode Gets the mode of the motor - brake or coast
     * @param hasForwardLimitSwitch Gets if there is any limit switch connected to the forward motor
     * @param hasForwardLimitSwitch Gets if there is any limit switch connected to the reverse motor
     * @param feedbackDevice Gets the feedback device for the motor controller.
     */
    public MA_TalonSRX(int id, boolean inverted, double rampRate, boolean mod, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, FeedbackDevice feedbackDevice) {
        talonSRX = new WPI_TalonSRX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mod);

        if (hasForwardLimitSwitch)
            configForwardLimitSwitchSource();

        if (hasReverseLimitSwitch)
            configReverseLimitSwitchSource();

        talonSRX.configSelectedFeedbackSensor(feedbackDevice);
    }


    /**
     * Constructing a new instance of TalonSRX class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param rampRate Gets the ramp rate to set to the motor
     * @param mode Gets the mode of the motor - brake or coast
     * @param hasForwardLimitSwitch Gets if there is any limit switch connected to the forward motor
     * @param hasForwardLimitSwitch Gets if there is any limit switch connected to the reverse motor
     */
    public MA_TalonSRX(int id, boolean inverted, double rampRate, boolean mode, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch) {
        talonSRX = new WPI_TalonSRX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mode);

        if (hasForwardLimitSwitch)
            configForwardLimitSwitchSource();

        if (hasReverseLimitSwitch)
            configReverseLimitSwitchSource();

    }


    /**
     * Constructing a new instance of TalonSRX class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param mode Gets the mode of the motor - brake or coast
     */
    public MA_TalonSRX(int id, boolean inverted, boolean mode) {
        talonSRX = new WPI_TalonSRX(id);
        setInverted(inverted);
        changeMode(mode);
    }

    /**
     * Configures a forward limit switch for a local/remote source
     */
    public void configForwardLimitSwitchSource() {
        talonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }


    /**
     * Configures a reverse limit switch for a local/remote source
     */
    public void configReverseLimitSwitchSource() {
        talonSRX.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }


    /**
     * sets the motor voltage
     * @param voltage Gets the voltage that the motor will operate at
     */
    @Override
    public void setVoltage(double voltage) {
        talonSRX.setVoltage(voltage);
    }


    /**
     * sets the motor velocity
     * @param velocity Gets the velocity that the motor will operate at
     */
    @Override
    public void setPower(double power) {
        talonSRX.set(ControlMode.PercentOutput, power);

    }


    /**
     * get the motor velocity
     * @return motor velocity
     */
    @Override
    public double getOutput() {
        return talonSRX.get();
    }



    /**
     * Sets the ramp rate
     * @param rampRate Gets the ramp rate to be set
     */
    @Override
    public void configRampRate(double rampRate) {
        talonSRX.configOpenloopRamp(rampRate);
        talonSRX.configClosedloopRamp(rampRate);
    }


    /**
     * Sets the motor to be invert or not
     * @param setInverted Gets if sets the motor to be invert or not
     */
    @Override
    public void setInverted(Boolean setInverted) {
        talonSRX.setInverted(setInverted);
    }

    /**
     * Changes the mode of the motor - brake or coast
     * @param onOff Gets the mode of the motor to be set
     */
    @Override
    public void changeMode(boolean onOff) {
        talonSRX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);
    }



    /**
     * reset the motor encoder to 0 degrees
     */
    public void resetEncoder() {
        talonSRX.setSelectedSensorPosition(0);
    }


    /**
     * get if the forward limit switch is pressed
     * @return forward LimitSwitch is pressed
     */
    public boolean getForwardLimitSwitch() {
        return talonSRX.getSensorCollection().isFwdLimitSwitchClosed();
    }


    /**
     * get if the reverse limit switch is pressed
     * @return reverse LimitSwitch is pressed
     */
    public boolean getReverseLimitSwitch() {
        return talonSRX.getSensorCollection().isRevLimitSwitchClosed();
    }

    /**
     * get the motor position
     * @return the number of rotations of the motor
     */
    public double getPosition() {
        return talonSRX.getSelectedSensorPosition();
    }


    /**
     * get the motor velocity
     * @return the RPM(rounds per minute) of the motor
     */
    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity();
    }


    /**
     * get the motor controller stator current 
     * @return the motor controller's output in Amps
     */
    public double getStatorCurrent() {
        return talonSRX.getStatorCurrent();
    }


    /**
     * inverts encoder or analog sensor
     * @param phaseSensor the sensor you want to invert
     */
    public void phaseSensor(boolean PhaseSensor) {
        talonSRX.setSensorPhase(PhaseSensor);
    }

    /**
     * resets the forward limitswitch
     * @param limit to reset or not
     */
    public void resetOnLimitF(boolean limit) {
        talonSRX.configClearPositionOnLimitF(limit, 0);
    }

    /**
     * resets the reverse limitswitch
     * @param limit to reset or not
     */
    public void resetOnLimitR(boolean limit) {
        talonSRX.configClearPositionOnLimitR(limit, 0);
    }


    /**
     * turn of the limit switches
     * @param onOff to turn on or of
     */
    public void disableLimit(boolean onOff) {
        talonSRX.configLimitSwitchDisableNeutralOnLOS(onOff, 0);
    }


    /**
     * Sets the enable state for limit switches.
     * @param override to override the enable state for limit switches or not.
     */
    public void overrideLimitSwitches(boolean override) {
        talonSRX.overrideLimitSwitchesEnable(override);
    }


    /**
     * get the motor ID
     * @return motor ID
     */
    @Override
    public int getID() {
        return talonSRX.getDeviceID();
    }


    /**
     * get a objeect from the current motor controller type
     * @return objeect from the current motor controller type
     */
    private WPI_TalonSRX getWPITalonSRX() {
        return talonSRX;
    }

    /**
     * set a motor to follow
     * @param motor the motor you want to follow at
     */
    public void follow(MA_TalonSRX motor) {
        talonSRX.follow(motor.getWPITalonSRX());
    }

    /**
     * Select the feedback device for the motor controller.
     * @param fd FeedbackDevice
     */
    public void setFeedBack(FeedbackDevice fd) {
        talonSRX.configSelectedFeedbackSensor(fd);
    }


    /**
     * Supply current limiting
     * @param limit limits of the current
     */
    public void setCurrentLimit(int limit) {
        talonSRX.configContinuousCurrentLimit(limit);
        talonSRX.configPeakCurrentDuration(0);

    }
}