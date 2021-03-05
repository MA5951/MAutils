// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Balance;

/** Add your docs here. */
public class BalanceConstants {
    public static final String KSUBSYSTEM_NAME = "Balance";
    public static final int MOTOR = 0;

    public final static double KSPROCKET_RADIUS = 0; //TODO
    public final static double KMOTOR_GEAR = 0; //TODO
    public static final double KSUB_GEAR = KSPROCKET_RADIUS * KMOTOR_GEAR; 
    public static final double KSUB_MAS = 0; //TODO
    public final static double KMOTOR_FORCE = KSUB_GEAR; // KSUB_GEAR * (Motor stall torque * num of motor) = Motor force
    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE /KSUB_MAS;
    public static final double KBEST_RPM = 0; //  KSHOOTER_GEAR * Motor max free RPM 
}

