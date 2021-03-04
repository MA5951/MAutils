// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

/** Add your docs here. */
public class ChassisConstants {
    public static final String KSUBSYSTEM_NAME = "Chassis";
    public static final double KP_MAPATH_DISTANCE = 1;
    public static final double KI_MAPATH_DISTANCE = 0;
    public static final double KD_MAPATH_DISTANCE = 0;

    public static final double KP_MAPATH_ANGLE = 1.5e-1;
    public static final double KI_MAPATH_ANGLE = 0;
    public static final double KD_MAPATH_ANGLE = 1e-2;

    public static final double KP_VISION_ANGLE = 2.5e-2;
    public static final double KI_VISION_ANGLE = 8e-4;
    public static final double KD_VISION_ANGLE = 0.5e-3;

    public static final double KP_VISION_DISTANCE = 1.6e-2;
    public static final double KI_VISION_DISTANCE = 0;
    public static final double KD_VISION_DISTANCE = 0;

    public static final double KANGLE_PID_VISION_SET_INPUTRANGE = 44.5;
    public static final double KANGLE_PID_MAPATH_SET_INPUTRANGE = 180;

    public static final double KRAMP_RATE_AUTO = 0.35;

    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = 7.05206988015; // TODO

    public static final double KTICKS_PER_METER = 22000;
    public final static double KSPROCKET_RADIUS = 0;
    public final static double KMOTOR_GEAR = 0;

    public static final double KCHASSIS_GEAR = KSPROCKET_RADIUS * KMOTOR_GEAR; // TODO
    public final static double KMOTOR_FORCE = KCHASSIS_GEAR;
}
