package com.ma5951.utils.controllersConstants;

import java.util.function.Supplier;

public class ProfiledPIDControllerConstants {
    private Supplier<Double> KP;
    private Supplier<Double> KI;
    private Supplier<Double> KD;
    private Supplier<Double> KS;
    private Supplier<Double> KV;
    private Supplier<Double> KA;
    private Supplier<Double> positionTolerance;
    private Supplier<Double> velocityTolerance;

    public ProfiledPIDControllerConstants (Supplier<Double> positionTolerance, 
    Supplier<Double> velocityTolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD, Supplier<Double> KS, Supplier<Double> KV, Supplier<Double> KA) {
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.KS = KS;
        this.KV = KV;
        this.KA = KA;
        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
    }

    public ProfiledPIDControllerConstants (Supplier<Double> positionTolerance, 
    Supplier<Double> velocityTolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD, Supplier<Double> KS, Supplier<Double> KV) {
        this(positionTolerance, velocityTolerance, KP, KI, KD, KS, KV, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (Supplier<Double> positionTolerance, 
    Supplier<Double> velocityTolerance, Supplier<Double> KP, Supplier<Double> KI, Supplier<Double> KD) {
        this(positionTolerance, velocityTolerance, KP, KI, KD, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (Supplier<Double> positionTolerance, 
    Supplier<Double> velocityTolerance, Supplier<Double> KP, Supplier<Double> KI) {
        this(positionTolerance, velocityTolerance, KP, KI, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (Supplier<Double> positionTolerance, 
    Supplier<Double> velocityTolerance, Supplier<Double> KP) {
        this(positionTolerance, velocityTolerance, KP, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (double positionTolerance, 
    double velocityTolerance, double KP, double KI, double KD, double KS, double KV, double KA) {
        this(()-> positionTolerance, ()-> velocityTolerance, ()-> KP, ()-> KI, ()-> KD, ()-> KS, ()-> KV, ()-> KA);
    }

    public ProfiledPIDControllerConstants (double positionTolerance, 
    double velocityTolerance, double KP, double KI, double KD, double KS, double KV) {
        this(()-> positionTolerance, ()-> velocityTolerance, ()-> KP, ()-> KI, ()-> KD, ()-> KS, ()-> KV, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (double positionTolerance, 
    double velocityTolerance, double KP, double KI, double KD) {
        this(()-> positionTolerance, ()-> velocityTolerance, ()-> KP, ()-> KI, ()-> KD, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (double positionTolerance, 
    double velocityTolerance, double KP, double KI) {
        this(()-> positionTolerance, ()-> velocityTolerance, ()-> KP, ()-> KI, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }

    public ProfiledPIDControllerConstants (double positionTolerance, 
    double velocityTolerance, double KP) {
        this(()-> positionTolerance, ()-> velocityTolerance, ()-> KP, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0);
    }

    public double getKD() {
        return KD.get();
    }

    public double getKI() {
        return KI.get();
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

    public double getPositionTolerance() {
        return positionTolerance.get();
    }

    public double getVelocityTolerance() {
        return velocityTolerance.get();
    }
}
