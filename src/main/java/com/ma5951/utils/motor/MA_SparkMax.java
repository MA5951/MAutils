// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Class that represents the SparkMax motor controller and its functions
 */

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
public class MA_SparkMax implements MotorController, MotorSensors {
    private final CANSparkMax canSparkMax;
    private RelativeEncoder canEncoder;
    private SparkMaxLimitSwitch forwardLimitSwitch;
    private SparkMaxLimitSwitch reversLimitSwitch;

    /**
     * Constructing a new instance of SparkMax class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param rampRate Gets the ramp rate to set to the motor
     * @param mode Gets the mode of the motor - brake or coast
     * @param limitSwitch Gets if there is any limit switch connected to the motor, and their type
     * @param encoder Gets if there is any encoder connected to the motor, and their type
     * @param type Get the motor type - brushed or brushless
     */

    public MA_SparkMax(int id, boolean inverted, double rampRate, boolean mode, RobotConstants.LIMIT_SWITCH limitSwitch,
            RobotConstants.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        configRampRate(rampRate);
        changeMode(mode);
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

    /**
     * Constructing a new instance of SparkMax class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param mode Gets the mode of the motor - brake or coast
     * @param limitSwitch Gets if there is any limit switch connected to the motor, and their type
     * @param encoder Gets if there is any encoder connected to the motor, and their type
     * @param type Get the motor type - brushed or brushless
     */

    public MA_SparkMax(int id, boolean inverted, boolean mode, RobotConstants.LIMIT_SWITCH limitSwitch,
            RobotConstants.ENCODER encoder, MotorType type) {
        canSparkMax = new CANSparkMax(id, type);
        canSparkMax.restoreFactoryDefaults();
        setInverted(inverted);
        changeMode(mode);
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

    /**
     * Constructing a new instance of SparkMax class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param rampRate Gets the ramp rate to set to the motor
     * @param mode Gets the mode of the motor - brake or coast
     * @param encoder Gets if there is any encoder connected to the motor, and their type
     * @param type Get the motor type - brushed or brushless
     */

    public MA_SparkMax(int id, boolean inverted, double rampRate, boolean mode, RobotConstants.ENCODER encoder,
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

    /**
     * Constructing a new instance of SparkMax class
     * @param id Gets the id of the motor controller
     * @param inverted Gets if the motor is inverted or not
     * @param mode Gets the mode of the motor - brake or coast
     * @param encoder Gets if there is any encoder connected to the motor, and their type
     * @param type Get the motor type - brushed or brushless
     */

    public MA_SparkMax(int id, boolean inverted, boolean mode, RobotConstants.ENCODER encoder, MotorType type) {
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

    /**
     * Sets the encoder's ticks per pulse
     */

    private void setCanEncoder() {
        canEncoder = canSparkMax.getEncoder();
        canEncoder.setPositionConversionFactor(RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setVelocityConversionFactor(RobotConstants.KTICKS_PER_PULSE);
    }

    /**
     * Sets the alternate encoder's ticks per pulse
     */

    private void setCanAlternateEncoder() {
        canEncoder = canSparkMax.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,
                RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setPositionConversionFactor(RobotConstants.KTICKS_PER_PULSE);
        canEncoder.setVelocityConversionFactor(RobotConstants.KTICKS_PER_PULSE);
    }

    /**
     * Sets the motor voltage
     * @param voltage Gets the voltage that the motor will operate at
     */

    @Override
    public void setVoltage(double voltage) {
        canSparkMax.setVoltage(voltage);
    }

    /**
     * Sets the motor power
     * @param power Gets the power value that the motor will operate at
     */

    @Override
    public void setPower(double power) {
        canSparkMax.set(power);

    }

    /**
     * Gets the output of the motor
     * @return The current output of the motor
     */

    @Override
    public double getOutput() {
        return canSparkMax.get();
    }

    /**
     * Sets the ramp rate
     * @param rampRate Gets the ramp rate to be set
     */

    @Override
    public void configRampRate(double rampRate) {
        canSparkMax.setOpenLoopRampRate(rampRate);
        canSparkMax.setClosedLoopRampRate(rampRate);

    }

    /**
     * Sets the motor to be invert or not
     * @param setInverted Gets if sets the motor to be invert or not
     */

    @Override
    public void setInverted(Boolean setInverted) {
        canSparkMax.setInverted(setInverted);

    }

    /**
     * Changes the mode of the motor - brushed or brashless
     * @param onOff Gets the mode of the motor to be set
     */

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

    private CANSparkMax getCanSparkMax() {
        return canSparkMax;
    }

    public void follow(MA_SparkMax motor) {
        canSparkMax.follow(motor.getCanSparkMax());
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