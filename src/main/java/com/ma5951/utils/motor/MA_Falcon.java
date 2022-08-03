package com.ma5951.utils.motor;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ma5951.utils.RobotConstants;

public class MA_Falcon implements MotorController, MotorSensors {

    private TalonFX falcon;

    /**
     * @param id
     * @param inverted
     * @param rampRate
     * @param mode
     * @param hasForwardLimitSwitch
     * @param hasReverseLimitSwitch
     * @param feedbackDevice
     */
    public MA_Falcon(int id, boolean inverted, double rampRate, NeutralMode mode, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, FeedbackDevice feedbackDevice) {
        falcon = new TalonFX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        falcon.setNeutralMode(mode);

        if (hasForwardLimitSwitch)
            configForwardLimitSwitchSource();

        if (hasReverseLimitSwitch)
            configReverseLimitSwitchSource();

        falcon.configSelectedFeedbackSensor(feedbackDevice);

    }

    public MA_Falcon(int id, boolean inverted, double rampRate, NeutralMode mode, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch) {
        falcon = new TalonFX(id);
        setInverted(inverted);
        configRampRate(rampRate);
        falcon.setNeutralMode(mode);

        if (hasForwardLimitSwitch)
            configForwardLimitSwitchSource();

        if (hasReverseLimitSwitch)
            configReverseLimitSwitchSource();

    }

    public MA_Falcon(int id, boolean inverted, NeutralMode mode) {
        falcon = new TalonFX(id);
        setInverted(inverted);
        falcon.setNeutralMode(mode);

    }

    private void configForwardLimitSwitchSource() {
        falcon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    private void configReverseLimitSwitchSource() {
        falcon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    }

    @Override
    public void setVoltage(double voltage) {
        falcon.set(TalonFXControlMode.PercentOutput, voltage / RobotConstants.voltage);

    }

    @Override
    public void setPower(double power) {
        falcon.set(TalonFXControlMode.PercentOutput, power);

    }

    @Override
    public double getOutput() {
        return falcon.getMotorOutputPercent();
    }

    @Override
    public void configRampRate(double rampRate) {
        falcon.configOpenloopRamp(rampRate);
        falcon.configClosedloopRamp(rampRate);

    }

    public void setInverted(Boolean setInverted) {
        falcon.setInverted(setInverted);

    }

    @Override
    public void changeMode(boolean onOff) {
        falcon.setNeutralMode(onOff ? NeutralMode.Brake : NeutralMode.Coast);

    }

    public void resetEncoder() {
        falcon.setSelectedSensorPosition(0);
    }

    public boolean getForwardLimitSwitch() {
        return falcon.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    }

    public boolean getReverseLimitSwitch() {
        return falcon.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public double getPosition() {
        return falcon.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return falcon.getSelectedSensorVelocity();
    }

    public double getStatorCurrent() {
        return falcon.getStatorCurrent();
    }

    public void phaseSensor(boolean PhaseSensor) {
        falcon.setSensorPhase(PhaseSensor);
    }

    public void resetOnLimitF(boolean limit) {
        falcon.configClearPositionOnLimitF(limit, 0);
    }

    public void resetOnLimitR(boolean limit) {
        falcon.configClearPositionOnLimitR(limit, 0);
    }

    public void disableLimit(boolean onOff) {
        falcon.configLimitSwitchDisableNeutralOnLOS(onOff, 0);
    }

    public void overrideLimitSwitches(boolean override) {
        falcon.overrideLimitSwitchesEnable(override);
    }

    @Override
    public int getID() {
        return falcon.getDeviceID();
    }

    private TalonFX getTalonFX() {
        return falcon;
    }

    public void follow(MA_Falcon motor) {
        falcon.follow(motor.getTalonFX());
    }

    public void setFeedBack(FeedbackDevice fd) {
        falcon.configSelectedFeedbackSensor(fd);
    }

    public void setCurrentLimit(int limit) {
        falcon.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, limit, RobotConstants.KTRIGGER_THRESHOLD_TIME, 0));
    }

    public void setCurrentLimit(int limit, int triggerThresholdCurrent) {
        falcon.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, limit,
                RobotConstants.KTRIGGER_THRESHOLD_TIME, triggerThresholdCurrent));
    }
}
