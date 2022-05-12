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
    private double tolorance;
    private double high;
    private double low;

    public PIDControllerConstans (double KP, double KI, double KD, double KF,
     double KS, double tolorance, double high, double low) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KF = KF;
        this.KS = KS;
        this.tolorance = tolorance;
        this.high = high;
        this.low = low;
    }

    public PIDControllerConstans (double KP, double KI, double KD, double KF,
     double tolorance, double high, double low) {
        this(KP, KI, KD, KF, 0, tolorance, high, low);
    }

    public PIDControllerConstans (double KP, double KI, double KD,
     double tolorance, double high, double low) {
        this(KP, KI, KD, 0, 0, tolorance, high, low);
    }

    public PIDControllerConstans (double KP, double KF,
     double tolorance, double high, double low) {
        this(KP, 0, 0, KF, 0, tolorance, high, low);
    }

    public PIDControllerConstans (double KP, double tolorance, double high, double low) {
        this(KP, 0, 0, 0, 0, tolorance, high, low);
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

    public double getTolorance() {
        return tolorance;
    }
}
