// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.MASubsystem.SubConstants;

/** Add your docs here. */
public class ShooterConstants extends SubConstants {
    public final static DCMotor MOTOR = DCMotor.getNEO(2);
    private final static double MAX_FREE_RPM = MOTOR.freeSpeedRadPerSec * 10;

    public final static double MOTOR_A_KP = 0;
    public final static double MOTOR_A_KI = 0;
    public final static double MOTOR_A_KD = 0;

    public final static int MOTOR_A = 0;
    public final static int MOTOR_B = 1;

    public static final String KSUBSYSTEM_NAME = "Shooter";
    public static final double KSHOOT_ANGLE = 0; // TODO
    public static final double KDELTA_Y = 0; // TODO

    public final static double KSPROCKET_RADIUS = 2.3776;
    public final static double KMOTOR_GEAR = 1 / 1.778;
    public static final double KSHOOTER_GEAR = KSPROCKET_RADIUS * KMOTOR_GEAR;
    public final static double KMOTOR_FORCE = KSHOOTER_GEAR * MOTOR.stallTorqueNewtonMeters;
    public static final double KSUB_MAS = 0.8; // KG
    public static final double KMAX_RPM = MAX_FREE_RPM * KMOTOR_GEAR;
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE / KSUB_MAS;
    public static final double KBEST_VOLTEG = MACalculations.RPMToVolteg(KMAX_RPM, KSPROCKET_RADIUS, KMOTOR_GEAR,
            KSUB_MAS);
    private static final double FlywhellRadius = 0.0508;
    public static final double kFlywheelMomentOfInertia = KSUB_MAS * Math.pow(FlywhellRadius, 2); // mas*R^2

}
