// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Calculation;

import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utils.RobotConstants;

public class MACalculations {

    public static double fromRPMToLinearSpeed(double RPM, double gear) {
        return (RPM * 10 ) * gear;
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

    public static double RPMToVolteg(double RPM, double sprocketRadius, double gear, double mas) {
        return MathUtil.clamp(
                ((fromRPMToLinearSpeed(RPM, gear) * mas) / RobotConstants.KDELTA_TIME * sprocketRadius * gear/RobotConstants.KELECTRON_CHARGE), 12, -12);
    }

}
