// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.motor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

/**
 * Add your docs here.
 */
public class VictorSPX implements MotorController {

    private WPI_VictorSPX victorSPX;

    public VictorSPX(int id, boolean inverted, double rampRate, boolean mode) {
        victorSPX = new WPI_VictorSPX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mode);
    }

    public VictorSPX(int id, boolean inverted, boolean mode) {
        victorSPX = new WPI_VictorSPX(id);
        setInverted(inverted);
        changeMode(mode);
    }

    @Override
    public void setVoltage(double voltage) {
        victorSPX.setVoltage(voltage);
    }

    @Override
    public void setPower(double power) {
        victorSPX.set(ControlMode.PercentOutput, power);
    }

    @Override
    public double getOutput() {
        return victorSPX.get();
    }

    @Override
    public void configRampRate(double rampRate) {
        victorSPX.configOpenloopRamp(rampRate);
        victorSPX.configClosedloopRamp(rampRate);

    }

    @Override
    public void setInverted(Boolean setInverted) {
        victorSPX.setInverted(setInverted);

    }

    @Override
    public void changeMode(boolean onOff) {
        victorSPX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);

    }

    public void follow(IMotorController motor) {
        victorSPX.follow(motor);
    }

    @Override
    public int getID() {
        return victorSPX.getDeviceID();
    }

}