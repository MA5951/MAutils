// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Calculation.MACalculations;

/** Add your docs here. */
public class ElevatorConstants {
    public final static DCMotor MOTOR = DCMotor.getNEO(1);
    private final static double MAX_FREE_RPM = MOTOR.freeSpeedRadPerSec * 10;

    public static final double KP_ELEVATOR_MOVE = 0; // TODO
    public static final double KI_ELEVATOR_MOVE = 0; // TODO
    public static final double KD_ELEVATOR_MOVE = 0; // TODO
    public static final double KF_ELEVATOR_MOVE = 0; //TODO

    public static final double KUP_SETPOINT = 100;
    public static final double KDOWN_SETPOINT = -100;

    public static final double KdrumRadius = 1;

    public static final int KELEVATOR_MOVE = 0;
    public static final String KSUBSYSTEM_NAME = "Elevator";

    public final static double KSPROCKET_RADIUS = 1; // TODO
    public final static double KMOTOR_GEAR = 1 / 11;

    public static final double KELEVATOR_GEAR = KMOTOR_GEAR * KSPROCKET_RADIUS;
    public final static double KMOTOR_FORCE = KELEVATOR_GEAR * MOTOR.stallTorqueNewtonMeters;
    public static final double KSUB_MAS = 1; // TODO
    public static final double KMAX_RPM = MAX_FREE_RPM * KMOTOR_GEAR;
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE / KSUB_MAS;
    public static final double KF = MACalculations
            .ForceToVoltage(KMOTOR_FORCE - (KSUB_MAS * RobotConstants.KGRAVITY_ACCELERATION));
}