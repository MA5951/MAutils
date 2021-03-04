// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SingleMotor;

/** Add your docs here. */
public class SingleMotorConstants {
    public static final String KSUBSYSTEM_NAME = "SingleMotor";
    public static final int MOTOR = 0;

    public final static double KSPROCKET_RADIUS = 0;
    public final static double KMOTOR_GEAR = 0;
    public static final double KSUB_GEAR = KSPROCKET_RADIUS * KMOTOR_GEAR; // TODO
    public final static double KMOTOR_FORCE = KSUB_GEAR; //TODO
}
