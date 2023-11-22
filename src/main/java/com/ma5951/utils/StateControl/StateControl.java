// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateControl extends SubsystemBase {
  private static StateControl statecontrol;

  public StateControl() {

  }


  public static StateControl getInstance() {
    if (statecontrol == null) {
      statecontrol = new StateControl();  
    }
    return statecontrol;
  }

  @Override
  public void periodic() {


  }
}
