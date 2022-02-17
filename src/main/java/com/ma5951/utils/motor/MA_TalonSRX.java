// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.motor;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MA_TalonSRX implements MotorController, MotorSensors {
    private WPI_TalonSRX talonSRX;

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

    public MA_TalonSRX(int id, boolean inverted, boolean mode) {
        talonSRX = new WPI_TalonSRX(id);
        setInverted(inverted);
        changeMode(mode);

    }

    public void configForwardLimitSwitchSource() {
        talonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    public void configReverseLimitSwitchSource() {
        talonSRX.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    @Override
    public void setVoltage(double voltage) {
        talonSRX.setVoltage(voltage);

    }

    @Override
    public void setPower(double power) {
        talonSRX.set(ControlMode.PercentOutput, power);

    }

    @Override
    public double getOutput() {
        return talonSRX.get();
    }

    @Override
    public void configRampRate(double rampRate) {
        talonSRX.configOpenloopRamp(rampRate);
        talonSRX.configClosedloopRamp(rampRate);

    }

    @Override
    public void setInverted(Boolean setInverted) {
        talonSRX.setInverted(setInverted);
    }

    @Override
    public void changeMode(boolean onOff) {
        talonSRX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);

    }

    public void resetEncoder() {
        talonSRX.setSelectedSensorPosition(0);
    }

    public boolean getForwardLimitSwitch() {
        return talonSRX.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimitSwitch() {
        return talonSRX.getSensorCollection().isRevLimitSwitchClosed();
    }

    public double getPosition() {
        return talonSRX.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return talonSRX.getSelectedSensorVelocity();
    }

    public double getStatorCurrent() {
        return talonSRX.getStatorCurrent();
    }

    public void phaseSensor(boolean PhaseSensor) {
        talonSRX.setSensorPhase(PhaseSensor);
    }

    public void resetOnLimitF(boolean limit) {
        talonSRX.configClearPositionOnLimitF(limit, 0);
    }

    public void resetOnLimitR(boolean limit) {
        talonSRX.configClearPositionOnLimitR(limit, 0);
    }

    public void disableLimit(boolean onOff) {
        talonSRX.configLimitSwitchDisableNeutralOnLOS(onOff, 0);
    }

    public void overrideLimitSwitches(boolean override) {
        talonSRX.overrideLimitSwitchesEnable(override);
    }

    @Override
    public int getID() {
        return talonSRX.getDeviceID();
    }

    private WPI_TalonSRX getWPITalonSRX() { return talonSRX; }

    public void follow(MA_TalonSRX motor) {
        talonSRX.follow(motor.getWPITalonSRX());
    }

    public void setFeedBack(FeedbackDevice fd) {
         talonSRX.configSelectedFeedbackSensor(fd);
    }

    public void setCurrentLimit(int limit) {
        talonSRX.configContinuousCurrentLimit(limit);
        talonSRX.configPeakCurrentDuration(0);

    }
}