// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

/** Add your docs here. */
public class ChassisConstants {
    public static final String SubsystemName = "Chassis";
    public static final double KP_MApath_distance = 1;
    public static final double KI_MApath_distance = 0;
    public static final double KD_MApath_distance = 0;

    public static final double KP_MApath_angle = 1.5e-1;
    public static final double KI_MApath_angle = 0;
    public static final double KD_MApath_angle = 1e-2;

    public static final double KP_Vision_angle = 2.5e-2;
    public static final double KI_Vision_angle = 8e-4;
    public static final double KD_Vision_angle = 0.5e-3;

    public static final double KP_Vision_distance = 1.6e-2;
    public static final double KI_Vision_distance = 0;
    public static final double KD_Vision_distance = 0;

    public static final double anglePIDVisionSetInputRange = 44.5;
    public static final double anglePidMApathSetInputRange = 180;

    public static final double Neo_RPM = 5700; //TODO
    public static final double Ramp_Rate_Auto = 0.35;

    public static final double Max_Speed = 12.9603246; //TODO
    public static final double Max_acceleration = 7.05206988015; //TODO

    public static final double ticksPerMeter = 22000;
}
