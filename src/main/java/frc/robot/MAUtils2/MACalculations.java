// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MAUtils2;

import edu.wpi.first.math.MathUtil;

public class MACalculations {
    private final static double KELECTRON_CHARGE = -1.6e-19;

    public static double fromRPMToLinearSpeed(double RPM, double gear) {
        return (RPM / 10) * gear;
    }

    public static double fromLinearSpeedToRPM(double linearSpeed, double gear) {
        return (linearSpeed / gear) * 10;
    }

    public static double toRPMFromEncoder(double Rate, int TPR) {
        return (Rate / TPR) / 60;
    }

    public static double toRPMFromEncoderConnectToTalon(double Rate, int TPR) {
        return ((Rate * 10) / TPR) / 60;
    }

    public static double ForceToVoltage(double force) {
        return MathUtil.clamp(force / KELECTRON_CHARGE, 12, -12);
    }

    public static double RPMToVoltage(double RPM, double maxRPM) {
        return (RPM / maxRPM) * 12;
    }
}