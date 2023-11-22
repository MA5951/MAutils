// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class StateRunner extends CommandBase {
  
  private State states[];
  private State currentState;
  private boolean isStateActiv;

  public StateRunner(State states[]) {
    this.states = states;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     currentState = new State(() -> true, new NullCommand() );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentState.runState();
    
    isStateActiv = false;
    for(int i = 0; i< states.length;i++){
      if (states[i].getConditionState()) {
        currentState = states[i];
        isStateActiv = true;
      }
    }
    
    if (!isStateActiv) {
      currentState = new State(() -> true, new NullCommand() );
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
