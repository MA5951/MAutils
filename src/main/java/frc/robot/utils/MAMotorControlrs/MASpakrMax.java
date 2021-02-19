// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAMotorControlrs;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.RobotConstants;

/** Add your docs here. */
class MASpakrMax implements MAMotorControlInterface {

    private CANSparkMax canSparkMax;
    private CANEncoder canEncoder;
    private CANDigitalInput ForwardLimitSwitch;
    private CANDigitalInput ReversLimitSwitch;

    public MASpakrMax(int ID, MotorType type) {
        canSparkMax = new CANSparkMax(ID, type);
        setCurrentLimit(60);
    }

    public MASpakrMax(int ID, boolean Inverted, MotorType type) {
        canSparkMax = new CANSparkMax(ID, type);
        setInverted(Inverted);
        setCurrentLimit(60);
    }

    public MASpakrMax(int ID, boolean Inverted, double rampRate, boolean mod, MotorType type) {
        canSparkMax = new CANSparkMax(ID, type);
        setInverted(Inverted);
        configRampRate(rampRate);
        changeMood(mod);
        setCurrentLimit(60);
    }

    public MASpakrMax(int ID, boolean Inverted, double rampRate, boolean mod, MASubsystem.ENCODER encoder,
            MotorType type) {
        canSparkMax = new CANSparkMax(ID, type);
        setInverted(Inverted);
        configRampRate(rampRate);
        changeMood(mod);
        setCurrentLimit(60);
        if (encoder == MASubsystem.ENCODER.Encoder) {
            setCanEncoder();
        } else {
            setCanAlternateEncoder();
        }
    }

    public MASpakrMax(int ID, boolean Inverted, double rampRate, boolean mod, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, MotorType type) {
        canSparkMax = new CANSparkMax(ID, type);
        setInverted(Inverted);
        configRampRate(rampRate);
        changeMood(mod);
        setCurrentLimit(60);
        if (hasForwardLimitSwitch)
            ForwardLimitSwitch = canSparkMax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (hasReverseLimitSwitch)
            ReversLimitSwitch = canSparkMax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    }

    public MASpakrMax(int ID, boolean Inverted, double rampRate, boolean mod, boolean hasForwardLimitSwitch,
            boolean hasReverseLimitSwitch, MASubsystem.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(ID, type);
        setInverted(Inverted);
        configRampRate(rampRate);
        changeMood(mod);
        setCurrentLimit(60);
        if (hasForwardLimitSwitch)
            ForwardLimitSwitch = canSparkMax.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (hasReverseLimitSwitch)
            ReversLimitSwitch = canSparkMax.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        if (encoder == MASubsystem.ENCODER.Encoder) {
            setCanEncoder();
        } else {
            setCanAlternateEncoder();
        }
    }

    private void setCanEncoder() {
        canEncoder = canSparkMax.getEncoder();
        canEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
        canEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
    }

    private void setCanAlternateEncoder() {
        canEncoder = canSparkMax.getAlternateEncoder(AlternateEncoderType.kQuadrature, RobotConstants.tiksPerPulse);
        canEncoder.setPositionConversionFactor(RobotConstants.tiksPerPulse);
        canEncoder.setVelocityConversionFactor(RobotConstants.tiksPerPulse);
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
        return ForwardLimitSwitch.get();
    }

    @Override
    public boolean getReversLimitSwitch() {
        return ReversLimitSwitch.get();
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
    public void PhaseSensor(boolean PhaseSensor) {
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
        ReversLimitSwitch.enableLimitSwitch(enable);
    }

    public void enableLimitSwitchF(boolean enable) {
        ForwardLimitSwitch.enableLimitSwitch(enable);
    }

    @Override
    public void setCurrentLimit(int limit) {
        canSparkMax.setSmartCurrentLimit(limit);
    }

}
