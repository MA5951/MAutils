// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.controllersConstants;

import java.util.function.Supplier;

/** Add your docs here. */
public class PIDControllerConstants {
    private Supplier<Double> KP;
    private Supplier<Double> KI;
    private Supplier<Double> KD;
    private Supplier<Double> KF;
    private Supplier<Double> KS;
    private Supplier<Double> KV;
    private Supplier<Double> KA;
    private Supplier<Double> tolerance;

    public PIDControllerConstants(Supplier<Double> tolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD, 
    Supplier<Double> KS, Supplier<Double> KV, Supplier<Double> KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = ()-> 0.0;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
        this.tolerance = tolerance;
    }

    public PIDControllerConstants(Supplier<Double> tolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD, 
    Supplier<Double> KS, Supplier<Double> KV) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = ()-> 0.0;
        this.KS = KS;
        this.KV = KV;
        this.KA = ()-> 0.0;
        this.tolerance = tolerance;
    }

    public PIDControllerConstants(Supplier<Double> tolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD, Supplier<Double> KF){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
        this.KS = ()-> 0.0;
        this.KV = ()-> 0.0;
        this.KA = ()-> 0.0;
        this.tolerance = tolerance;
    }

    public PIDControllerConstants (Supplier<Double> tolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD){
        this(tolerance, KP, KI, KD, ()-> 0.0);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     */
    public PIDControllerConstants (Supplier<Double> tolerance, Supplier<Double> KP, Supplier<Double> KI){
        this(tolerance, KP, KI, ()-> 0.0, ()-> 0.0, ()-> 1.0, ()-> -1.0);
    }

    /**
     * @param tolerance
     * @param KP
     */
    public PIDControllerConstants (Supplier<Double> tolerance, Supplier<Double> KP){
        this(tolerance, KP, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 1.0, ()-> -1.0);
    }
    
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
        double KS, double KV, double KA){
        this(()-> tolerance, ()-> KP, ()-> KI, ()-> KD, ()-> KS, ()-> KV, ()-> KA);
    }

    public PIDControllerConstants (double tolerance, double KP, double KI, double KD,
     double KS, double KV){
        this(()-> tolerance, ()-> KP, ()-> KI, ()-> KD, ()-> KS, ()-> KV);
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
    public PIDControllerConstants (double tolerance, double KP, double KI, double KD, double KF) {
        this(()-> tolerance, ()-> KP, ()-> KI, ()-> KD, ()-> KF);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     * @param KD
     */
    public PIDControllerConstants (double tolerance, double KP, double KI, double KD){
        this(()-> tolerance, ()-> KP, ()-> KI, ()-> KD, ()-> 0.0);
    }

    /**
     * @param tolerance
     * @param KP
     * @param KI
     */
    public PIDControllerConstants (double tolerance, double KP, double KI){
        this(()-> tolerance, ()-> KP, ()-> KI, ()-> 0.0, ()-> 0.0);
    }

    /**
     * @param tolerance
     * @param KP
     */
    public PIDControllerConstants (double tolerance, double KP){
        this(()-> tolerance, ()-> KP, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }
    
    public double getKD() {
        return KD.get();
    }

    public double getKI() {
        return KI.get();
    }

    public double getKF() {
        return KF.get();
    }

    public double getKP() {
        return KP.get();
    }

    public double getKS() {
        return KS.get();
    }

    public double getKV() {
        return KV.get();
    }

    public double getKA() {
        return KA.get();
    }

    public double gettolerance() {
        return tolerance.get();
    }
}

