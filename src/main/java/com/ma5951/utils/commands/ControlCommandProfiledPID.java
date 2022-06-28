// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.controllers.ProfiledPIDControllerConstants;
import com.ma5951.utils.subsystem.ControlSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlCommandProfiledPID extends CommandBase {
  private ControlSubsystem subsystem;
  private Supplier<TrapezoidProfile.State> goal;
  private ProfiledPIDController  ProfiledPID;
  private SimpleMotorFeedforward feedforward;
  private double lastTime;
  private double lastSpeed;
  private boolean AtGoal;
  private double delay;
  private double time;

  public ControlCommandProfiledPID(ControlSubsystem subsystem, Supplier<TrapezoidProfile.State> goal,
   double maxVelocity, double maxAcceleration,
    ProfiledPIDControllerConstants profiledpidControllerConstant, double delay) {
    this.delay = delay;
    this.subsystem = subsystem;
    this.goal = goal;
    this.feedforward = new SimpleMotorFeedforward(profiledpidControllerConstant.getKS(),
      profiledpidControllerConstant.getKV(), profiledpidControllerConstant.getKA());
    ProfiledPID = new ProfiledPIDController(profiledpidControllerConstant.getKP(),
     profiledpidControllerConstant.getKI(), profiledpidControllerConstant.getKD(),
     new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    ProfiledPID.setTolerance(profiledpidControllerConstant.getPositionTolerance(),
      profiledpidControllerConstant.getVelocityTolerance());
    addRequirements(subsystem);
  }

  public ControlCommandProfiledPID (ControlSubsystem subsystem, TrapezoidProfile.State goal,
    double maxVelocity, double maxAcceleration,
    ProfiledPIDControllerConstants profiledpidControllerConstant, double delay){
      this(subsystem, () -> goal, maxVelocity, maxAcceleration, profiledpidControllerConstant, delay);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ProfiledPID.setGoal(goal.get());
    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double acceleration = (ProfiledPID.getSetpoint().velocity - lastSpeed) / 
    (Timer.getFPGATimestamp() - lastTime);
    subsystem.setVoltage(
      ProfiledPID.calculate(subsystem.getMeasurement())
      + feedforward.calculate(ProfiledPID.getSetpoint().velocity, acceleration)
    );
    lastTime = Timer.getFPGATimestamp();
    lastSpeed = ProfiledPID.getSetpoint().velocity;
    if (ProfiledPID.atGoal() && AtGoal){
      AtGoal = true;
      time = Timer.getFPGATimestamp();
    }
    if (!ProfiledPID.atGoal()) {
      AtGoal = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ProfiledPID.atGoal() && (Timer.getFPGATimestamp() - time) >= delay) || !subsystem.canMove();
  }
}
