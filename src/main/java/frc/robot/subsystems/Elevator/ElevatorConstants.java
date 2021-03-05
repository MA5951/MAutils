// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

/** Add your docs here. */
public class ElevatorConstants {

    public static final double KP_ELEVATOR_MOVE = 0; // TODO
    public static final double KI_ELEVATOR_MOVE = 0; // TODO
    public static final double KD_ELEVATOR_MOVE = 0; // TODO
    public static final double KF_ELEVATOR_MOVE = 0;

    public static final int ELEVATOR_MOVE = 0;
    public static final String KSUBSYSTEM_NAME = "Elevator";
   

    public final static double KSPROCKET_RADIUS = 0; //TODO
    public final static double KMOTOR_GEAR = 0; //TODO
    public static final double KELEVATOR_GEAR = KMOTOR_GEAR * KSPROCKET_RADIUS; 
    public final static double KMOTOR_FORCE = KELEVATOR_GEAR; //* Motor stall torque * num of motor = Motor force
    public static final double KSUB_MAS = 0; //TODO
    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE / KSUB_MAS; 
    public static final double KBEST_RPM = 0; //  KSHOOTER_GEAR * Motor max free RPM
}
