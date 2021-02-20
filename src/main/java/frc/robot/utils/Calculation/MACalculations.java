// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Calculation;


import frc.robot.utils.RobotConstants;

public class MACalculations {
    private static double curentV = 0;
    private static double prev_v = 0;

    public static double FromRPMToLinearSpeed(double RPM, double Gear) {
       
        return (RPM * 10) * Gear;
    }

    public static double FromLinearSpeedToRPM(double LinearSpeed, double Gear) {
        return (LinearSpeed * Gear) / 10;
    }

    public static double ToRPMFromEncoder(double Rate, int TPR) {
        return (Rate / TPR) / 60;
    }

    public static double ToRPMFromEncoderConnectToTalon(double Rate, int TPR) {
        return ((Rate * 10) / TPR) / 60;
    }

    private static double DeltaV(double RPM, double Gear) {
        curentV = FromRPMToLinearSpeed(RPM, Gear) / RobotConstants.KDELTA_TIME;
        double delta = curentV - prev_v;
        prev_v = curentV;
        return delta;

    }

    public static double Acceleration(double RPM, double Gear) {
        return DeltaV(RPM, Gear) / RobotConstants.KDELTA_TIME;
    }
}
