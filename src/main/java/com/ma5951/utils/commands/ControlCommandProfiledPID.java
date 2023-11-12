// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.controllersConstants.ProfiledPIDControllerConstants;
import com.ma5951.utils.subsystem.ControllableSubsystem;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlCommandProfiledPID extends CommandBase {
  private ControllableSubsystem subsystem;
  private Supplier<TrapezoidProfile.State> goal;
  private ProfiledPIDController ProfiledPID;
  private SimpleMotorFeedforward feedforward;
  private double lastTime;
  private double lastSpeed;
  private boolean AtGoal;
  private double delay;
  private double time;
  private ProfiledPIDControllerConstants profiledPIDConstants;
  private double maxVelocity;
  private double maxAcceleration;
  private boolean needToStopGivingPowerAtTheEnd;

  /**
   * @param goal needs to be position or velocity and position
   * @param delay the amount of time you what the system to be in the goal before stoping
   */
  public ControlCommandProfiledPID(ControllableSubsystem subsystem, Supplier<TrapezoidProfile.State> goal,
   double maxVelocity, double maxAcceleration,
    ProfiledPIDControllerConstants profiledpidControllerConstants, double delay,
    boolean needToStopGivingPowerAtTheEnd) {
    this.delay = delay;
    this.subsystem = subsystem;
    this.goal = goal;
    this.profiledPIDConstants = profiledpidControllerConstants;
    this.maxAcceleration = maxAcceleration;
    this.maxVelocity = maxVelocity;
    this.needToStopGivingPowerAtTheEnd = needToStopGivingPowerAtTheEnd;
    addRequirements(subsystem);
  }

   /**
   * @param goal needs to be position or velocity and position
   * @param delay the amount of time you what the system to be in the goal before stoping
   */
  public ControlCommandProfiledPID (ControllableSubsystem subsystem, TrapezoidProfile.State goal,
    double maxVelocity, double maxAcceleration,
    ProfiledPIDControllerConstants profiledpidControllerConstant, double delay,
    boolean needToStopGivingPowerAtTheEnd){
      this(subsystem, () -> goal, maxVelocity, maxAcceleration,
        profiledpidControllerConstant, delay, needToStopGivingPowerAtTheEnd);
  }
  
  /**
   * @param goal needs to be position or velocity and position
   */
  public ControlCommandProfiledPID (ControllableSubsystem subsystem, TrapezoidProfile.State goal,
   double maxVelocity, double maxAcceleration,
   ProfiledPIDControllerConstants profiledpidControllerConstant) {
    this(subsystem, () -> goal, maxVelocity, maxAcceleration,
    profiledpidControllerConstant, 0, true);
  }
  
  /**
   * @param goal needs to be position or velocity and position
   */
  public ControlCommandProfiledPID (ControllableSubsystem subsystem, Supplier<TrapezoidProfile.State> goal, double maxVelocity, 
    double maxAcceleration, ProfiledPIDControllerConstants profiledpidControllerConstant) {
    this(subsystem, goal, maxVelocity, maxAcceleration,
      profiledpidControllerConstant, 0, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();
    this.feedforward = new SimpleMotorFeedforward(profiledPIDConstants.getKS(),
    profiledPIDConstants.getKV(), profiledPIDConstants.getKA());
    ProfiledPID = new ProfiledPIDController(profiledPIDConstants.getKP(),
    profiledPIDConstants.getKI(), profiledPIDConstants.getKD(),
     new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    ProfiledPID.setTolerance(profiledPIDConstants.getPositionTolerance(),
    profiledPIDConstants.getVelocityTolerance());
    ProfiledPID.setGoal(goal.get());
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
    if (ProfiledPID.atGoal() && !AtGoal){
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
    if (needToStopGivingPowerAtTheEnd) {
      subsystem.setPower(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ProfiledPID.atGoal() && (Timer.getFPGATimestamp() - time) >= delay) || !subsystem.canMove();
  }
}
