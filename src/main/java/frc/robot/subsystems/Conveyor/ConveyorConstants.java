// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.utils.Calculation.MACalculations;

/** Add your docs here. */
public class ConveyorConstants {
    private final static DCMotor KTRANSPORTATION = DCMotor.getBag(1);
    private final static double MAX_FREE_RPM_TRANSPORTATION = KTRANSPORTATION.freeSpeedRadPerSec * 10;

    private final static DCMotor KCONVEYOR = DCMotor.getBag(1);
    private final static double MAX_FREE_RPM_CONVEYOR = KCONVEYOR.freeSpeedRadPerSec * 10;

    public static final String KSUBSYSTEM_NAME = "Conveyor";
    public static final int KTRANSPORTATION_MOTOR = 1;
    public static final int KCONVEYOR_MOTOR = 0;

    public static final double KGEAR_TRANSPORTATION_MOTOR = 1 / 10; // TODO
    public static final double KGEAR_KCONVEYOR_MOTOR = 1 / 10; // TODO

    public static final double KTRANSPORTATION_SPROCKET_RADIUS = 0;
    public static final double KCONVEYOR_SPROCKET_RADIUS = 0;

    public final static double KMOTOR_TRANSPORTATION_FORCE = KGEAR_TRANSPORTATION_MOTOR
            * KTRANSPORTATION.stallTorqueNewtonMeters;

    public final static double KMOTOR_CONVEYOR_FORCE = KGEAR_KCONVEYOR_MOTOR * KCONVEYOR.stallTorqueNewtonMeters;

    public static final double KTRANSPORTATION_MAS = 0;
    public static final double KCONVEYOR_MAS = 0;

    public static final double KMAX_TRANSPORTATION_RPM = MAX_FREE_RPM_TRANSPORTATION * KGEAR_TRANSPORTATION_MOTOR;
    public static final double KMAX_TRANSPORTATION_ACCELERATION = KMOTOR_TRANSPORTATION_FORCE / KTRANSPORTATION_MAS;

    public static final double KMAX_CONVEYOR_RPM = MAX_FREE_RPM_CONVEYOR * KGEAR_KCONVEYOR_MOTOR;
    public static final double KMAX_CONVEYOR_ACCELERATION = KMOTOR_CONVEYOR_FORCE / KCONVEYOR_MAS;

    public static final double KBEST_TRANSPORTATION_VOLTEG = MACalculations.RPMToVolteg(KMAX_TRANSPORTATION_RPM,
            KTRANSPORTATION_SPROCKET_RADIUS, KGEAR_TRANSPORTATION_MOTOR, KTRANSPORTATION_MAS);

    public static final double KBEST_CONVEYOR_VOLTEG = MACalculations.RPMToVolteg(KMAX_CONVEYOR_RPM,
            KCONVEYOR_SPROCKET_RADIUS, KGEAR_KCONVEYOR_MOTOR, KCONVEYOR_MAS);
}
