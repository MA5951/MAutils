// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Calculation;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utils.RobotConstants;

public class MACalculations {

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

    public static double RPMToVolteg(double RPM, double sprocketRadius, double gear, double mas) {
        return MathUtil.clamp(
                ((FromRPMToLinearSpeed(RPM, gear) * mas) / RobotConstants.KDELTA_TIME * sprocketRadius * gear/C), 12, -12);
    }

}
