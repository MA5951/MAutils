// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

/** Add your docs here. */
public class ShooterConstants {
    public final static double MOTOR_A_KP = 0;
    public final static double MOTOR_A_KI = 0;
    public final static double MOTOR_A_KD = 0;

    public final static int MOTOR_A = 0;
    public static final String KSUBSYSTEM_NAME = "Shooter";
    public static final double KSHOOT_ANGLE = 0; // TODO
    public static final double KDELTA_Y = 0; // TODO

    public final static double KSPROCKET_RADIUS = 0; // TODO
    public final static double KMOTOR_GEAR = 0; // TOOD
    public static final double KSHOOTER_GEAR = KSPROCKET_RADIUS * KMOTOR_GEAR; // TODO
    public final static double KMOTOR_FORCE = KSHOOTER_GEAR; // * Motor stall torque * num of motor = Motor force
    public static final double KSUB_MAS = 0; // TOOD
    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE / KSUB_MAS; // TODO
    public static final double KBEST_RPM = 0; // KSHOOTER_GEAR * Motor max free RPM

}
