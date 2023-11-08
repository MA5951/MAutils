// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class StateRunner extends CommandBase {
  
  private State states[];
  private int currentState;

  public StateRunner(State states[]) {
    this.states = states;
  }

  public void setCurrentState(int currentState) {
    this.currentState = currentState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    states[currentState].runState();
    
    for(int i = 0; i< states.length;i++){
      if (states[i].getConditionState()) {
        currentState = i;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
