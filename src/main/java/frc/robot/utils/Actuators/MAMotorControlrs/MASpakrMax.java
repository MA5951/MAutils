// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Actuators.MAMotorControlrs;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;

/** Add your docs here. */
class MASpakrMax implements MAMotorControlInterface {

    private CANSparkMax canSparkMax;
    private CANEncoder canEncoder;
    private CANDigitalInput forwardLimitSwitch;
    private CANDigitalInput reversLimitSwitch;

    public MASpakrMax(int id, boolean inverted, double rampRate, boolean mod, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, MASubsystem.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        configRampRate(rampRate);
        changeMood(mod);
        setCurrentLimit(60);
        if (hasForwardLimitSwitch)
            forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (hasReverseLimitSwitch)
            reversLimitSwitch = canSparkMax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (encoder == MASubsystem.ENCODER.Encoder) {
            setCanEncoder();
        } else if (encoder == MASubsystem.ENCODER.Alternate_Encoder) {
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
    public void changeMood(boolean onOff) {
        canSparkMax.setIdleMode(onOff ? IdleMode.kBrake : IdleMode.kCoast);

    }

    @Override
    public void resetEncoder() {
        canEncoder.setPosition(0);
    }

    @Override
    public boolean getForwardLimitSwitch() {
        return forwardLimitSwitch.get();
    }

    @Override
    public boolean getReversLimitSwitch() {
        return reversLimitSwitch.get();
    }

    @Override
    public double getPosition() {
        return canEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return canEncoder.getVelocity();
    }

    @Override
    public double getStatorCurrent() {
        return canSparkMax.getOutputCurrent();
    }

    @Override
    public void phaseSensor(boolean PhaseSensor) {
        canEncoder.setInverted(PhaseSensor);
    }

    @Override
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

    @Override
    public void setCurrentLimit(int limit) {
        canSparkMax.setSmartCurrentLimit(limit);
    }

}
