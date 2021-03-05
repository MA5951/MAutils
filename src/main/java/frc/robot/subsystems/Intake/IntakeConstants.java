// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

/** Add your docs here. */
public class IntakeConstants {
    public static final double KP_INTAKE_MOVE = 0; // TODO
    public static final double KI_INTAKE_MOVE = 0; // TODO
    public static final double KD_INTAKE_MOVE = 0; // TODO

    public static final int INTAKE_MOVE = 1;
    public static final int INTAKE_COLLECTION = 0;
    public static final String KSUBSYSTEM_NAME = "Intake";

    public final static double KSPROCKET_RADIUS_MOVE = 0;// TODO
    public final static double KMOTOR_GEAR_MOVE = 0;// TODO
    public static final double KGEAR_INTAKE_MOVE = KSPROCKET_RADIUS_MOVE * KMOTOR_GEAR_MOVE;
    public final static double KMOTOR_MOVE_FORCE = KGEAR_INTAKE_MOVE; //* Motor stall torque * num of motor = Motor force

    public final static double KSPROCKET_RADIUS_COLLECTION = 0;// TODO
    public final static double KMOTOR_GEAR_COLLECTION = 0;// TODO
    public static final double KGEAR_INTAKE_COLLECTION = KSPROCKET_RADIUS_COLLECTION * KMOTOR_GEAR_COLLECTION;
    public final static double KMOTOR_COLLECTION_FORCE = KGEAR_INTAKE_COLLECTION; //* Motor stall torque * num of motor = Motor force
    public static final double KSUB_MAS = 0;

    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_MOVE_FORCE / KSUB_MAS ; // TODO
    public static final double KBEST_RPM_COLLECTION = 0; //  KSHOOTER_GEAR * Motor max free RPM
    public static final double KBEST_RPM_MOVE = 0; //  KSHOOTER_GEAR * Motor max free RPM
}
