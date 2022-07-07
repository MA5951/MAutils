// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Making calculations
 */

package com.ma5951.utils;

import edu.wpi.first.math.MathUtil;

public class Calculations {
    private final static double K_ELECTRON_CHARGE = -1.6e-19;

    /**
     * Converting from RPM to linear speed
     * @param RPM The RPM to convert
     * @param gear The gear that attached to the motor
     * @return The linear speed
     */

    public static double fromRPMToLinearSpeed(double RPM, double gear) {
        return (RPM / 10) * gear;
    }

    /**
     * Converting from linear speed to RPM
     * @param linearSpeed The linear speed to convert
     * @param gear The gear that attached to the motor
     * @return The RPM
     */

    public static double fromLinearSpeedToRPM(double linearSpeed, double gear) {
        return (linearSpeed / gear) * 10;
    }

    /**
     * Converting from encoder to RPM
     * @param Rate The econder value to convert
     * @param TPR The encoder's ticks per round
     * @return The RPM
     */

    public static double toRPMFromEncoder(double Rate, int TPR) {
        return (Rate / TPR) / 60;
    }

    /**
     * Convert from encoder that connected to Talon to RPM
     * @param Rate The encoder value to convert
     * @param TPR The ecnoder's ticks per round
     * @return The RPM
     */

    public static double toRPMFromEncoderConnectToTalon(double Rate, int TPR) {
        return ((Rate * 10) / TPR) / 60;
    }

    /**
     * Convert from force to voltage
     * @param force The force to convert
     * @return The Voltage
     */

    public static double ForceToVoltage(double force) {
        return MathUtil.clamp(force / K_ELECTRON_CHARGE, 12, -12);
    }

    /**
     * Convert from RPM to voltage
     * @param RPM The RPM to convert
     * @param maxRPM The motor's maximum RPM
     * @return The voltage
     */

    public static double RPMToVoltage(double RPM, double maxRPM) {
        return (RPM / maxRPM) * 12;
    }
}