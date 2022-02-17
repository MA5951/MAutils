// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.controllers.interfaces;

/**
 * Add your docs here.
 */
public interface BaseController {
    public void setSetpoint(double setpoint);

    public boolean atSetpoint();

    public double calculate(double measurement, double setpoint);

    public double calculate(double measurement);

    public double getPositionError();

    public double getSetpoint();

    public void reset();

}
