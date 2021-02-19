// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAMotorControlrs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


/** Add your docs here. */
 class MAVictorSPX implements MAMotorControlInterface {

    private WPI_VictorSPX victorSPX;

    public MAVictorSPX(int ID) {
        victorSPX = new WPI_VictorSPX(ID);
    }

    public MAVictorSPX(int ID, boolean Inverted) {
        victorSPX = new WPI_VictorSPX(ID);
        setInverted(Inverted);
    }

    public MAVictorSPX(int ID, boolean Inverted, double rampRate, boolean mod) {
        victorSPX = new WPI_VictorSPX(ID);
        setInverted(Inverted);
        configRampRate(rampRate);
        changeMood(mod);
    }

    @Override
    public void setvoltage(double voltage) {
        victorSPX.setVoltage(voltage);
    }

    @Override
    public void set(double Power) {
        victorSPX.set(ControlMode.PercentOutput, Power);
    }

    @Override
    public double getOutPut() {
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
    public void changeMood(boolean onOff) {
        victorSPX.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);

    }

    public IMotorController getIMotorController() {
        return victorSPX;
    }

    public void follow(IMotorController Motor) {
        victorSPX.follow(Motor);
    }

    @Override
    public int getID() {
        return victorSPX.getDeviceID();
    }

}
