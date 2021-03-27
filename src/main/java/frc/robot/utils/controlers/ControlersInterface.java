// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Controlers;

/** Add your docs here. */
public interface ControlersInterface {
public void setSetpoint(double setPoint);
public boolean atSetpoint();
public boolean atSetpoint(double waitTime);
public double calculate(double measurement, double setPoint);
public double calculate(double measurement);
public double getPositionError();
public double getSetpoint();
public void reset();

}
