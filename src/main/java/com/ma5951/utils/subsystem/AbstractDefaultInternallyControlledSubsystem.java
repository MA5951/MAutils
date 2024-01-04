// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.subsystem;

import java.util.function.Supplier;

import com.ma5951.utils.MAShuffleboard;
import com.ma5951.utils.MAShuffleboard.pidControllerGainSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class AbstractDefaultInternallyControlledSubsystem 
    extends SubsystemBase implements DefaultInternallyControlledSubsystem{

    private double setPoint;
    private double tolorance;
    protected MAShuffleboard shuffleboard;
    protected pidControllerGainSupplier pidGainSupplier;
    private Supplier<Double> getMeasurement;

    public AbstractDefaultInternallyControlledSubsystem(
        double tolorance, String tabName, Supplier<Double> getMeasurment){
        this.tolorance = tolorance;
        shuffleboard = new MAShuffleboard(tabName);
        pidGainSupplier = shuffleboard.getPidControllerGainSupplier();
        this.getMeasurement = getMeasurment;
    }

    public boolean atPoint(){
        return Math.abs(this.getMeasurement.get() - setPoint)
         <= this.tolorance;
    }

    public double getSetPoint(){
        return this.setPoint;
    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public void periodic() {
        shuffleboard.addNum("Set Point", getSetPoint());
        shuffleboard.addBoolean("At Point", atPoint());
        shuffleboard.addNum("tolorance", this.tolorance);
        shuffleboard.addBoolean("can move", canMove());
      }
    

}
