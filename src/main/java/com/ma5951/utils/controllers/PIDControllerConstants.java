// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.controllers;

/** Add your docs here. */
public class PIDControllerConstants {
    private double KP;
    private double KI;
    private double KD;
    private double KF;
    private double KS;
    private double KV;
    private double KA;
    private double tolerance;
    private double high;
    private double low;

    /**
     * @param tolerance
     * @param KP
     * @param KI
     * @param KD
     * @param KS
     * @param KV
     * @param KA
     * @param high
     * @param low
     */
    public PIDControllerConstants (double tolerance, double KP, double KI, double KD, 
        double low, double high, double KS, double KV, double KA){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = 0;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
        this.tolerance = tolerance;
        this.high = high;
        this.low = low;
    }

    public PIDControllerConstants (double tolerance, double KP, double KI, double KD, 
        double low, double high, double KS, double KV){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = 0;
        this.KS = KS;
        this.KV = KV;
        this.KA = 0;
        this.tolerance = tolerance;
        this.high = high;
        this.low = low;
    }


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
    public PIDControllerConstants (double tolerance, double KP, double KI, double KD, double KF,
        double high, double low) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
        this.KS = 0;
        this.KV = 0;
        this.KA = 0;
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
     */
    public  PIDControllerConstants (double tolerance, double KP, double KI, double KD, double KF){
        this(tolerance, KP, KI, KD, KF, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     * @param KD
     */
    public  PIDControllerConstants (double tolerance, double KP, double KI, double KD){
        this(tolerance, KP, KI, KD, 0, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     */
    public  PIDControllerConstants (double tolerance, double KP, double KI){
        this(tolerance, KP, KI, 0, 0, 1, -1);
    }

    /**
     * @param tolerance
     * @param KP
     */
    public  PIDControllerConstants (double tolerance, double KP){
        this(tolerance, KP, 0, 0, 0, 1, -1);
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

    public double getKV() {
        return KV;
    }

    public double getKA() {
        return KA;
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

