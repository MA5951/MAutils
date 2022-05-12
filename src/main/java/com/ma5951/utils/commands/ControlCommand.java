// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.ma5951.utils.controllers.PIDController;
import com.ma5951.utils.controllers.PIDControllerConstans;
import com.ma5951.utils.subsystem.ControlSubsystem;

public class ControlCommand extends CommandBase {
  /** Creates a new MAControlCommand. */

  private ControlSubsystem subsystem;
  private Supplier<Double> setpoint;
  private boolean voltage;
  private PIDController pid;
  private double delay;
  private double time;
  private boolean wasInSetPoint;


  public ControlCommand(ControlSubsystem subsystem, Supplier<Double> setpoint,
   boolean voltage, double delay, PIDControllerConstans PIDConstans) {
    this.subsystem = subsystem;
    this.setpoint = setpoint;
    this.voltage = voltage;
    this.delay = delay;
    pid = new PIDController(PIDConstans.getKP(), PIDConstans.getKI(),
     PIDConstans.getKD(), PIDConstans.getKF(), PIDConstans.getTolorance(), 
     PIDConstans.getLow(), PIDConstans.getHigh());

    addRequirements(subsystem);
  }

  public ControlCommand(ControlSubsystem subsystem, double setpoint,
   boolean voltage, double delay, PIDControllerConstans PIDConstans) {
    this(subsystem, () -> setpoint, voltage, delay, PIDConstans);
  }

  public ControlCommand(ControlSubsystem subsystem, double setpoint,
   boolean voltage, PIDControllerConstans PIDConstans) {
    this(subsystem, () -> setpoint, voltage, 0, PIDConstans);
  }

  public ControlCommand(ControlSubsystem subsystem, Supplier<Double> setpoint,
   boolean voltage, PIDControllerConstans PIDConstans) {
    this(subsystem, setpoint, voltage, 0, PIDConstans);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wasInSetPoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (voltage) {
        subsystem.setVoltage(pid.calculate(setpoint.get()));
      } else {
        subsystem.setPower(pid.calculate(setpoint.get()));
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
