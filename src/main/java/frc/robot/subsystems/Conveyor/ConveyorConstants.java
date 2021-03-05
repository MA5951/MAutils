// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyor;

/** Add your docs here. */
public class ConveyorConstants {
    public static final String KSUBSYSTEM_NAME = "Conveyor";
    public static final int KTRANSPORTATION_MOTOR = 1;
    public static final int KCONVEYOR_MOTOR = 0;

    public static final double KGEAR_TRANSPORTATION_MOTOR = 0; // TODO
    public static final double KGEAR_KCONVEYOR_MOTOR = 0; // TODO
    public final static double KMOTOR_FORCE = KGEAR_KCONVEYOR_MOTOR; //* Motor stall torque * num of motor = Motor force
    public static final double KSUB_MAS = 0;
    public static final double KMAX_SPEED = 12.9603246; //TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE /KSUB_MAS; 
    public static final double KBEST_RPM = 0; //  KSHOOTER_GEAR * Motor max free RPM
}
