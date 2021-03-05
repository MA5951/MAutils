// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import frc.robot.utils.Calculation.MACalculations;

/** Add your docs here. */
public class IntakeConstants {
    public static final int INTAKE_COLLECTION = 0;
    public static final String KSUBSYSTEM_NAME = "Intake";

    public final static double KSPROCKET_RADIUS_COLLECTION = 0;// TODO
    public final static double KMOTOR_GEAR_COLLECTION = 0;// TODO
    public static final double KGEAR_INTAKE_COLLECTION = KSPROCKET_RADIUS_COLLECTION * KMOTOR_GEAR_COLLECTION;
    public final static double KMOTOR_COLLECTION_FORCE = KGEAR_INTAKE_COLLECTION; // * Motor stall torque * num of motor
                                                                                  // = Motor force
    public static final double KSUB_MAS = 0;

    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_GEAR_COLLECTION / KSUB_MAS; // TODO
    public static final double KBEST_RPM_COLLECTION = MACalculations.RPMToVolteg(0, KSPROCKET_RADIUS_COLLECTION,
            KMOTOR_GEAR_COLLECTION, KSUB_MAS); // KSHOOTER_GEAR * Motor max free RPM

}
