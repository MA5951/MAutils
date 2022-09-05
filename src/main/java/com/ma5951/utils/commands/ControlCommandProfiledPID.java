// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import java.util.function.Supplier;

import com.ma5951.utils.Shuffleboard;
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
  private boolean isShuffleboard;
  private Shuffleboard board;

  public ControlCommandProfiledPID(ControlSubsystem subsystem, String tabName) {
    this.subsystem = subsystem;
    isShuffleboard = true;
    addRequirements(subsystem);
    board = new Shuffleboard(tabName);
    board.addNum("setPointPosition", 0);
    board.addNum("setPointVelocity", 0);
    board.addNum("Kp", 0);
    board.addNum("Ki", 0);
    board.addNum("Kd", 0);
    board.addNum("Ks", 0);
    board.addNum("Kv", 0);
    board.addNum("Ka", 0);
    board.addNum("positionTolerance", 0);
    board.addNum("velocityTolerance", 0);
    board.addNum("delay", 0);
    board.addNum("maxVelocity", 0);
    board.addNum("maxAcceleration", 0);
  }

  /**
   * @param goal needs to be position or velocity and position
   * @param delay the amount of time you what the system to be in the goal before stoping
   */
  public ControlCommandProfiledPID(ControlSubsystem subsystem, Supplier<TrapezoidProfile.State> goal,
   double maxVelocity, double maxAcceleration,
    ProfiledPIDControllerConstants profiledpidControllerConstants, double delay) {
    this.delay = delay;
    this.subsystem = subsystem;
    this.goal = goal;
    this.feedforward = new SimpleMotorFeedforward(profiledpidControllerConstants.getKS(),
    profiledpidControllerConstants.getKV(), profiledpidControllerConstants.getKA());
    ProfiledPID = new ProfiledPIDController(profiledpidControllerConstants.getKP(),
    profiledpidControllerConstants.getKI(), profiledpidControllerConstants.getKD(),
     new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    ProfiledPID.setTolerance(profiledpidControllerConstants.getPositionTolerance(),
    profiledpidControllerConstants.getVelocityTolerance());
    addRequirements(subsystem);
  }

   /**
   * @param goal needs to be position or velocity and position
   * @param delay the amount of time you what the system to be in the goal before stoping
   */
  public ControlCommandProfiledPID (ControlSubsystem subsystem, TrapezoidProfile.State goal,
    double maxVelocity, double maxAcceleration,
    ProfiledPIDControllerConstants profiledpidControllerConstant, double delay){
      this(subsystem, () -> goal, maxVelocity, maxAcceleration, profiledpidControllerConstant, delay);
  }
  
  /**
   * @param goal needs to be position or velocity and position
   */
  public ControlCommandProfiledPID (ControlSubsystem subsystem, TrapezoidProfile.State goal,
   double maxVelocity, double maxAcceleration,
   ProfiledPIDControllerConstants profiledpidControllerConstant) {
    this(subsystem, () -> goal, maxVelocity, maxAcceleration, profiledpidControllerConstant, 0);
  }
  
  /**
   * @param goal needs to be position or velocity and position
   */
  public ControlCommandProfiledPID (ControlSubsystem subsystem, Supplier<TrapezoidProfile.State> goal, double maxVelocity, 
    double maxAcceleration, ProfiledPIDControllerConstants profiledpidControllerConstant) {
    this(subsystem, goal, maxVelocity, maxAcceleration, profiledpidControllerConstant, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastSpeed = 0;
    lastTime = Timer.getFPGATimestamp();
    if (isShuffleboard) {
      this.feedforward = new SimpleMotorFeedforward(board.getNum("Ks"),
       board.getNum("Kv"), board.getNum("Ka"));
      this.ProfiledPID = new ProfiledPIDController(board.getNum("Kp"), 
       board.getNum("Ki"), board.getNum("Kd"),
        new TrapezoidProfile.Constraints(board.getNum("maxVelocity"), board.getNum("maxAcceleration")));
      this.ProfiledPID.setGoal(new TrapezoidProfile.State(
        board.getNum("setPointPosition"), board.getNum("setPointVelocity")));
      this.ProfiledPID.setTolerance(board.getNum("positionTolerance"), board.getNum("velocityTolerance"));
      this.delay = board.getNum("delay");
    } else {
      ProfiledPID.setGoal(goal.get());
    }
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
    board.addNum("Measurement", subsystem.getMeasurement());
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
