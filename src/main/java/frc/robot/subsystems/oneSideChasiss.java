// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MAUtils2.MAShuffleboard;
import frc.robot.MAUtils2.MAMotorController.MAMotorAndSensorInterface;
import frc.robot.MAUtils2.MAMotorController.MATalonSRX;
import frc.robot.MAUtils2.MASubsystem.MotorInterface;

public class oneSideChasiss extends SubsystemBase implements MotorInterface {
  
  private MAMotorAndSensorInterface front;
  private MAMotorAndSensorInterface back;

  private static oneSideChasiss oneSide;

  private MAShuffleboard shuffleboard;

  public oneSideChasiss() {
    front = new MATalonSRX(1, false, 0, false, false, false, FeedbackDevice.QuadEncoder);
    back = new MATalonSRX(2, false, 0, false, false, false, FeedbackDevice.QuadEncoder);

    shuffleboard = new MAShuffleboard("Test");
  }
  
  @Override
  public void setVoltege(double voltege) {
    front.setvoltage(voltege);
    back.setvoltage(voltege);
  }
  
  @Override
  public double getVoltege() {
    return front.getOutPut();
  }

  public double getEncoder(){
    return front.getPosition();
  }


  public static oneSideChasiss getInstance() {
    if (oneSide == null){
      oneSide = new oneSideChasiss();
    }
    return oneSide;
  }

  @Override
  public void periodic() {
    // shuffleboard.addNum("Front Motor", getVoltege());
    // shuffleboard.addNum("Front Encoder", getVoltege());
    shuffleboard.addNum("Encoder:", getEncoder());
  }
}
