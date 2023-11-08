// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.subsystem;

/** Add your docs here. */
public abstract class AbstractDefaultInternallyControlledSubsystem 
implements DefaultInternallyControlledSubsystem{

    private double setPoint;
    private double tolerance;

    public double getSetPoint(){
        return this.setPoint;
    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public boolean atPoint(){
        return Math.abs(getMeasurement() - setPoint) <= this.tolerance;
    }
}
