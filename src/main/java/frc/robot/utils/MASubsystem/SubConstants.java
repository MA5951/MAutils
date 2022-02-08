// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MASubsystem;

import java.util.ArrayList;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.Calculation.MACalculations;

/** Add your docs here. */
public class SubConstants {
    public ArrayList<DCMotor> motors = new ArrayList<>();
    public final double KmaxFreeRPM = motors.get(0).freeSpeedRadPerSec * 10;
    public String KSUBSYSTEM_NAME = "";
    public double KSprocketRadius = 1;
    public double KMotorGear = 1;
    public final double KSubGear = KSprocketRadius * KMotorGear;
    public final double KmotorForce = KSubGear * motors.get(0).stallTorqueNewtonMeters;
    public double KSubMas = 1; // KG
    public final double KMaxRPM = KmaxFreeRPM * KMotorGear;
    public final double KMaxAcceleration = KmotorForce / KSubMas;
    public final double KBestVoltage = MACalculations.RPMToVoltage(KMaxRPM, KmaxFreeRPM);
    public double KRaduisForMoment = 1;
    public final double KMomentOfInertia = KSubMas * Math.pow(KRaduisForMoment, 2); // mas*R^
}
