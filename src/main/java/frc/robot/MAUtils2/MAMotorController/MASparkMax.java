// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2.MAMotorController;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.MAUtils2.RobotConstants;

/** Add your docs here. */
class MASparkMax implements MAMotorControlInterface {

    private CANSparkMax canSparkMax;
    private CANEncoder canEncoder;
    private CANDigitalInput forwardLimitSwitch;
    private CANDigitalInput reversLimitSwitch;

    public MASparkMax(int id, boolean inverted, double rampRate, boolean mod, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, RobotConstants.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mod);
        setCurrentLimit(60);
        if (hasForwardLimitSwitch)
            forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (hasReverseLimitSwitch)
            reversLimitSwitch = canSparkMax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (encoder == RobotConstants.ENCODER.Encoder) {
            setCanEncoder();
        } else if (encoder == RobotConstants.ENCODER.Alternate_Encoder) {
            setCanAlternateEncoder();
        }
    }

    private void setCanEncoder() {
        canEncoder = canSparkMax.getEncoder();
        canEncoder.setPositionConversionFactor(RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setVelocityConversionFactor(RobotConstants.KTICKS_PER_PULSE);
    }

    private void setCanAlternateEncoder() {
        canEncoder = canSparkMax.getAlternateEncoder(AlternateEncoderType.kQuadrature, RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setPositionConversionFactor(RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setVelocityConversionFactor(RobotConstants.KTICKS_PER_PULSE);
    }

    @Override
    public void setvoltage(double voltage) {
        canSparkMax.setVoltage(voltage);
    }

    @Override
    public void set(double Power) {
        canSparkMax.set(Power);

    }

    @Override
    public double getOutPut() {
        return canSparkMax.get();
    }

    @Override
    public void configRampRate(double rampRate) {
        canSparkMax.setOpenLoopRampRate(rampRate);
        canSparkMax.setClosedLoopRampRate(rampRate);

    }

    @Override
    public void setInverted(Boolean setInverted) {
        canSparkMax.setInverted(setInverted);

    }

    @Override
    public void changeMode(boolean onOff) {
        canSparkMax.setIdleMode(onOff ? IdleMode.kBrake : IdleMode.kCoast);

    }

    public void resetEncoder() {
        canEncoder.setPosition(0);
    }

    public boolean getForwardLimitSwitch() {
        return forwardLimitSwitch.get();
    }

    public boolean getReverseLimitSwitch() {
        return reversLimitSwitch.get();
    }

    public double getPosition() {
        return canEncoder.getPosition();
    }

    public double getVelocity() {
        return canEncoder.getVelocity();
    }

    public double getStatorCurrent() {
        return canSparkMax.getOutputCurrent();
    }

    public void phaseSensor(boolean PhaseSensor) {
        canEncoder.setInverted(PhaseSensor);
    }

    public int getID() {
        return canSparkMax.getDeviceId();
    }

    public CANSparkMax getCanSparkMax() {
        return canSparkMax;
    }

    public void follow(CANSparkMax Motor) {
        canSparkMax.follow(Motor);
    }

    public void enableLimitSwitchR(boolean enable) {
        reversLimitSwitch.enableLimitSwitch(enable);
    }

    public void enableLimitSwitchF(boolean enable) {
        forwardLimitSwitch.enableLimitSwitch(enable);
    }

    public void setCurrentLimit(int limit) {
        canSparkMax.setSmartCurrentLimit(limit);
    }

}