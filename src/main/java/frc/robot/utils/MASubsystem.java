/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.ArrayList;

import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;

/**
 * Add your docs here.
 */
public interface MASubsystem {
  public enum ENCODER {
    No_Encoder, Encoder, Alternate_Encoder
  }

  public enum MOTOR_CONTROLL {
    TALON, VICTOR, SPARKMAXBrushless, SPARKMAXBrushled
  }

  public enum IDMotor {
    ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8, ID9, ID10, ID11, ID12, ID13, ID14, ID15, ID16
  }

  public ArrayList<MAMotorControler> maMotorControlers = new ArrayList<MAMotorControler>();

  default void setMAMotorComtrolersList(MAMotorControler Motor) {
    maMotorControlers.add(Motor);
  }

  default void addFollowMotorToMaster(MAMotorControler Master, IDMotor id) {
    MAMotorControler Motor = new MAMotorControler(Master.getMotorControllType(), id);
    Motor.follow(Master);
  }

  public Runnable setMotorPower(double power, int Indax);

  public void PrintValues();

}
