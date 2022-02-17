// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ma5951.utils.RobotConstants;

/**
 * Add your docs here.
 */
public class SparkMax implements MotorController, MotorSensors {
    private final CANSparkMax canSparkMax;
    private RelativeEncoder canEncoder;
    private SparkMaxLimitSwitch forwardLimitSwitch;
    private SparkMaxLimitSwitch reversLimitSwitch;

    public SparkMax(int id, boolean inverted, double rampRate, boolean mod, RobotConstants.LIMIT_SWITCH limitSwitch,
            RobotConstants.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mod);
        setCurrentLimit(60);
        if (limitSwitch == RobotConstants.LIMIT_SWITCH.forward)
            forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        if (limitSwitch == RobotConstants.LIMIT_SWITCH.reverse)
            reversLimitSwitch = canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        if (encoder == RobotConstants.ENCODER.Encoder) {
            setCanEncoder();
        } else if (encoder == RobotConstants.ENCODER.Alternate_Encoder) {
            setCanAlternateEncoder();
        }
    }

    public SparkMax(int id, boolean inverted, boolean mod, RobotConstants.LIMIT_SWITCH limitSwitch,
            RobotConstants.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        changeMode(mod);
        setCurrentLimit(60);
        if (limitSwitch == RobotConstants.LIMIT_SWITCH.forward)
            forwardLimitSwitch = canSparkMax.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        if (limitSwitch == RobotConstants.LIMIT_SWITCH.reverse)
            reversLimitSwitch = canSparkMax.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        if (encoder == RobotConstants.ENCODER.Encoder) {
            setCanEncoder();
        } else if (encoder == RobotConstants.ENCODER.Alternate_Encoder) {
            setCanAlternateEncoder();
        }
    }

    public SparkMax(int id, boolean inverted, double rampRate, boolean mode, RobotConstants.ENCODER encoder,
            MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mode);
        setCurrentLimit(60);
        if (encoder == RobotConstants.ENCODER.Encoder) {
            setCanEncoder();
        } else if (encoder == RobotConstants.ENCODER.Alternate_Encoder) {
            setCanAlternateEncoder();
        }
    }

    public SparkMax(int id, boolean inverted, boolean mode, RobotConstants.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        changeMode(mode);
        setCurrentLimit(60);

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
        canEncoder = canSparkMax.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,
                RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setPositionConversionFactor(RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setVelocityConversionFactor(RobotConstants.KTICKS_PER_PULSE);
    }

    @Override
    public void setVoltage(double voltage) {
        canSparkMax.setVoltage(voltage);
    }

    @Override
    public void setPower(double power) {
        canSparkMax.set(power);

    }

    @Override
    public double getOutput() {
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
        return forwardLimitSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return reversLimitSwitch.isPressed();
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

    public void phaseSensor(boolean phaseSensor) {
        canEncoder.setInverted(phaseSensor);
    }

    public int getID() {
        return canSparkMax.getDeviceId();
    }

    public CANSparkMax getCanSparkMax() {
        return canSparkMax;
    }

    public void follow(CANSparkMax motor) {
        canSparkMax.follow(motor);
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