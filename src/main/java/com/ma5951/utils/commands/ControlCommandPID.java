// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.controllers.PIDControllerConstants;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ControlCommandPID extends CommandBase {
  /** Creates a new MAControlCommand. */

  private ControlSubsystem subsystem;
  private Supplier<Double> setpoint;
  private boolean isVoltage;
  private PIDController pid;
  private double delay;
  private double time;
  private boolean wasInSetPoint;
  private SimpleMotorFeedforward feedforward;
  private PIDControllerConstants pidConstants;

  /**
   * @param isVoltage if you want the motor to work in Voltage way and not a percentage way
   * @param delay the amount of time you what the system to be in the goal before stoping
   */
  public ControlCommandPID(ControlSubsystem subsystem, Supplier<Double> setpoint,
   PIDControllerConstants PIDConstants, boolean isVoltage, double delay) {
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.isVoltage = isVoltage;
    this.delay = delay;
    this.pidConstants = PIDConstants;
    addRequirements(subsystem);
  }

  /**
   * @param isVoltage if you want the motor to work in Voltage way and not a percentage way
   * @param delay the amount of time you what the system to be in the goal before stoping
   */
  public ControlCommandPID(ControlSubsystem subsystem, double setpoint,
  PIDControllerConstants PIDConstans, boolean isVoltage, double delay) {
    this(subsystem, () -> setpoint, PIDConstans, isVoltage, delay);
  }

  /**
   * @param isVoltage if you want the motor to work in Voltage way and not a percentage way
   */
  public ControlCommandPID(ControlSubsystem subsystem, double setpoint,
   PIDControllerConstants PIDConstans, boolean isVoltage) {
    this(subsystem, () -> setpoint, PIDConstans, isVoltage, 0);
  }

  /**
  * @param isVoltage if you want the motor to work in Voltage way and not a percentage way
  */
  public ControlCommandPID(ControlSubsystem subsystem, Supplier<Double> setpoint,
   PIDControllerConstants PIDConstans, boolean isVoltage) {
    this(subsystem, setpoint, PIDConstans, isVoltage, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.feedforward = new SimpleMotorFeedforward(pidConstants.getKS(),
    pidConstants.getKV(), pidConstants.getKA());
    pid = new PIDController(pidConstants.getKP(), pidConstants.getKI(),
    pidConstants.getKD(), pidConstants.getKF(), pidConstants.gettolerance(), 
    pidConstants.getLow(), pidConstants.getHigh());
    pid.setSetpoint(setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isVoltage) {
      subsystem.setVoltage(pid.calculate(subsystem.getMeasurement()) + feedforward.calculate(pid.getSetpoint()));
    } else {
      subsystem.setPower(pid.calculate(subsystem.getMeasurement()) + feedforward.calculate(pid.getSetpoint()));
    }
    if (pid.atSetpoint() && !wasInSetPoint){
      time = Timer.getFPGATimestamp();
      wasInSetPoint = true;
    }
    if (!pid.atSetpoint()){
      wasInSetPoint = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return (pid.atSetpoint() && (Timer.getFPGATimestamp() - time) >= delay) || !subsystem.canMove();
  }
}
