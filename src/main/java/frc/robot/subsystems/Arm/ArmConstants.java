// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

/** Add your docs here. */
public class ArmConstants {
    public static final double KP_ARM_MOVE = 0; // TODO
    public static final double KI_ARM_MOVE = 0; // TODO
    public static final double KD_ARM_MOVE = 0; // TODO
    public static final double KF_ARM_MOVE = 0; // TODO

    public static final double KP_SEC_ARM_MOVE = 0; // if use TwoJointedArm
    public static final double KI_SEC_ARM_MOVE = 0; // if use TwoJointedArm
    public static final double KD_SEC_ARM_MOVE = 0; // if use TwoJointedArm
    public static final double KF_SEC_ARM_MOVE = 0; // if use TwoJointedArm

    public static final String KSUBSYSTEM_NAME = "Arm";
    public static final int KARM_MOVE = 0;
    public static final int KSEC_ARM_MOVE = 0; // if use TwoJointedArm

    public final static double KSPROCKET_RADIUS = 0; //TOOD
    public final static double KMOTOR_GEAR = 0; //TODO
    public static final double KARM_GEAR = KSPROCKET_RADIUS * KMOTOR_GEAR; 
    public final static double KMOTOR_FORCE = KARM_GEAR; // * Motor stall torque * num of motor = Motor force
    public static final double KSUB_MAS = 0;
    public static final double KMAX_SPEED = 12.9603246; // TODO
    public static final double KMAX_ACCELERATION = KMOTOR_FORCE / KSUB_MAS; // TODO
    public static final double KBEST_RPM = 0; //  KSHOOTER_GEAR * Motor max free RPM

}
