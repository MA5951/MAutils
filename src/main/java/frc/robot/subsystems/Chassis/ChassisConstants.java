// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Calculation.MACalculations;

/** Add your docs here. */
public class ChassisConstants {
    public static final String KSUBSYSTEM_NAME = "Chassis";

    public static final double KP_MAPATH_RIGHT_VELOCITY = 0;
    public static final double KI_MAPATH_RIGHT_VELOCITY = 0;
    public static final double KD_MAPATH_RiGHT_VELOCITY = 0;

    public static final double KP_MAPATH_LEFT_VELOCITY = 0;
    public static final double KI_MAPATH_LEFT_VELOCITY = 0;
    public static final double KD_MAPATH_LEFT_VELOCITY = 0;

    public static final double KP_MAPATH_ANGLE = 0;
    public static final double KI_MAPATH_ANGLE = 0;
    public static final double KD_MAPATH_ANGLE = 0;

    public static final double KP_VISION_ANGLE = 2.5e-2;
    public static final double KI_VISION_ANGLE = 8e-4;
    public static final double KD_VISION_ANGLE = 0.5e-3;

    public static final double KP_VISION_DISTANCE = 1.6e-2;
    public static final double KI_VISION_DISTANCE = 0;
    public static final double KD_VISION_DISTANCE = 0;

    public static final double KANGLE_PID_VISION_SET_INPUTRANGE = 44.5;
    public static final double KANGLE_PID_MAPATH_SET_INPUTRANGE = 180;

    public static final double KRAMP_RATE_AUTO = 0.35;
    public static final double KTHRESHOLD = 0.1;
    public static final double KSCALE = 0.3;

    public static final double KchassisLength = 0.7; // TODO
    public static final double KwhellRadius = 0.0762;

    public static final double KSUB_MAS = 56;
    public static final int KTICKS_PER_METER = 22000;
    public final static double KSPROCKET_RADIUS = 0.0162671;// TOOD
    public final static double KMOTOR_GEAR = 1 / 11.25;
    public static final double KCHASSIS_GEAR = KwhellRadius * KMOTOR_GEAR; // TODO
    public final static double KMOTOR_FORCE = ((1 / KMOTOR_GEAR) *RobotConstants.KSTALL_TORQUE_NEO * 2) / KwhellRadius;
    public static final double KMAX_SPEED = MACalculations
            .fromRPMToLinearSpeed(DCMotor.getNEO(2).freeSpeedRadPerSec * 10 , KCHASSIS_GEAR); // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE / KSUB_MAS;// KMOTOR_FORCE / KSUB_MAS; // TODO
}
