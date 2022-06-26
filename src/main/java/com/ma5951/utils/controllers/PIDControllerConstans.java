// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.controllers;

/** Add your docs here. */
public class PIDControllerConstans {
    private double KP;
    private double KI;
    private double KD;
    private double KF;
    private double KS;
    private double tolerance;
    private double high;
    private double low;

    /**
     * @param KP
     * @param KI
     * @param KD
     * @param KF
     * @param KS
     * @param tolerance
     * @param high
     * @param low
     */
    public PIDControllerConstans (double tolerance, double KP, double KI, double KD, double KF,
        double KS, double high, double low) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
        this.KS = KS;
        this.tolerance = tolerance;
        this.high = high;
        this.low = low;
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     * @param KD
     * @param KF
     * @param KS
     */
    public  PIDControllerConstans (double tolerance, double KP, double KI, double KD, double KF,
        double KS){
            this(tolerance, KP, KI, KD, KF, KS, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     * @param KD
     * @param KF
     */
    public  PIDControllerConstans (double tolerance, double KP, double KI, double KD, double KF){
        this(tolerance, KP, KI, KD, KF, 0, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     * @param KD
     */
    public  PIDControllerConstans (double tolerance, double KP, double KI, double KD){
        this(tolerance, KP, KI, KD, 0, 0, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     */
    public  PIDControllerConstans (double tolerance, double KP, double KI){
        this(tolerance, KP, KI, 0, 0, 0, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     */
    public  PIDControllerConstans (double tolerance, double KP){
        this(tolerance, KP, 0, 0, 0, 0, 1, -1);
    }
    
    public double getKD() {
        return KD;
    }

    public double getKI() {
        return KI;
    }

    public double getKF() {
        return KF;
    }

    public double getKP() {
        return KP;
    }

    public double getKS() {
        return KS;
    }

    public double getHigh() {
        return high;
    }

    public double getLow() {
        return low;
    }

    public double gettolerance() {
        return tolerance;
    }
}
