// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.Calculation.MACalculations;

/** Add your docs here. */
public class IntakeConstants {
    private final static DCMotor MOTOR = DCMotor.getAndymarkRs775_125(1);
    private final static double MAX_FREE_RPM = MOTOR.freeSpeedRadPerSec * 10;

    public static final double KP_INTAKE_MOVE = 0; // TODO
    public static final double KI_INTAKE_MOVE = 0; // TODO
    public static final double KD_INTAKE_MOVE = 0; // TODO

    public static final int INTAKE_COLLECTION = 0;
    public static final int INTAKE_MOVE = 1;
    public static final String KSUBSYSTEM_NAME = "Intake";

    public final static double KSPROCKET_RADIUS_COLLECTION = 0;// TODO
    public final static double KMOTOR_GEAR_COLLECTION = 1 / 5;// TODO
    public static final double KGEAR_INTAKE_COLLECTION = KSPROCKET_RADIUS_COLLECTION * KMOTOR_GEAR_COLLECTION;
    public final static double KMOTOR_COLLECTION_FORCE = KGEAR_INTAKE_COLLECTION * MOTOR.stallTorqueNewtonMeters;
    public final static double KSUB_MAS = 0;
    public static final double KMAX_RPM = MAX_FREE_RPM * KMOTOR_GEAR_COLLECTION; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_GEAR_COLLECTION / KSUB_MAS; // TODO
    public static final double KBEST_RPM_COLLECTION = MACalculations.RPMToVoltage(KMAX_RPM, MAX_FREE_RPM);

}
