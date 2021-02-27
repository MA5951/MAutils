// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import frc.robot.utils.Calculation.MACalculations;
import frc.robot.utils.RobotConstants;

/** Add your docs here. */
public class ElevatorConstants {
    public static final double KGEAR = 0; // TODO
    public static final double KP_ELEVATOR_MOVE = 0; // TODO
    public static final double KI_ELEVATOR_MOVE = 0; // TODO
    public static final double KD_ELEVATOR_MOVE = 0; // TODO
    public static final double KF_ELEVATOR_MOVE = MACalculations
            .FromLinearSpeedToRPM((RobotConstants.KGRAVITY_ACCELERATION / 2) * RobotConstants.KDELTA_TIME, KGEAR);
    public static final int ELEVATOR_MOVE = 0;
    public static final String KSUBSYSTEM_NAME = "Elevator";
    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = 7.05206988015; // TODO
}
