// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

public class MACalculations {
    private static double curentV = 0;
    private static double prev_v = 0;

    public static double FromRPMToLinearSpeed(double RPM, double Gear) {
        return (RPM * 10) * Gear;
    }

    public static double FromLinearSpeedToRPM(double LinearSpeed, double Gear) {
        return (LinearSpeed * 10) / Gear;
    }

    public static double ToRPMFromEncoder(double Rate, int TPR) {
        return (Rate / TPR) / 60;
    }

    public static double ToRPMFromEncoderConnectTalon(double Rate, int TPR) {
        return ((Rate * 10) / TPR) / 60;
    }

    public static double CIMRPMToVolteg() {
        return 0; // TODO
    }

    public static double MiniCIMRPMToVolteg() {
        return 0; // TODO
    }

    public static double NeoRPMToVolteg() {
        return 0; // TODO
    }

    public static double ProRPMToVolteg() {
        return 0; // TODO
    }

    private static double DeltaV(double RPM, double Gear) {
        curentV = FromRPMToLinearSpeed(RPM, Gear) / RobotConstants.DeltaTime;
        double delta = curentV - prev_v;
        prev_v = curentV;
        return delta;

    }

    public static double Acceleration(double RPM, double Gear) {
        return DeltaV(RPM, Gear) / RobotConstants.DeltaTime;
    }
}
