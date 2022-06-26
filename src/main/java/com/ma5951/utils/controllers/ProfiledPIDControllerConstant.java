package com.ma5951.utils.controllers;

public class ProfiledPIDControllerConstant {
    private double KP;
    private double KI;
    private double KD;
    private double KS;
    private double KV;
    private double KA;
    private double positionTolerance;
    private double velocityTolerance;

    public ProfiledPIDControllerConstant (double positionTolerance, 
    double velocityTolerance, double KP, double KI, double KD, double KS, double KV, double KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    public ProfiledPIDControllerConstant (double positionTolerance, 
    double velocityTolerance, double KP, double KI, double KD, double KS, double KV) {
        this(positionTolerance, velocityTolerance, KP, KI, KD, KS, KV, 0);
    }

    public ProfiledPIDControllerConstant (double positionTolerance, 
    double velocityTolerance, double KP, double KI, double KD) {
        this(positionTolerance, velocityTolerance, KP, KI, KD, 0, 0, 0);
    }

    public ProfiledPIDControllerConstant (double positionTolerance, 
    double velocityTolerance, double KP, double KI) {
        this(positionTolerance, velocityTolerance, KP, KI, 0, 0, 0, 0);
    }

    public ProfiledPIDControllerConstant (double positionTolerance, 
    double velocityTolerance, double KP) {
        this(positionTolerance, velocityTolerance, KP, 0, 0, 0, 0, 0);
    }

    public double getKD() {
        return KD;
    }

    public double getKI() {
        return KI;
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

    public double getPositionTolerance() {
        return positionTolerance;
    }

    public double getVelocityTolerance() {
        return velocityTolerance;
    }
}
