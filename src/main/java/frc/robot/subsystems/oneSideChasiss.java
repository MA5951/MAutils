// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MAUtils2.MAMotorController.MAFalcon;
import frc.robot.MAUtils2.MAMotorController.MAMotorControlInterface;
import frc.robot.MAUtils2.MASubsystem.MotorInterface;

public class oneSideChasiss extends SubsystemBase implements MotorInterface {

  private MAMotorControlInterface front;
  private MAMotorControlInterface back;
  private oneSideChasiss subChassis;

  public oneSideChasiss() {
    front = new MAFalcon(0, false, 0, NeutralMode.Brake, false, false, FeedbackDevice.None);
    back = new MAFalcon(1, false, 0, NeutralMode.Brake, false, false, FeedbackDevice.None);
  }
  
  @Override
  public void setVoltege(double voltege) {
    front.setvoltage(voltege);
    back.setvoltage(voltege);
  }
  
  @Override
  public double getPower() {
    return front.getOutPut();
  }

  @Override
  public oneSideChasiss getInstance() {
    if (subChassis == null){
      subChassis = new oneSideChasiss();
    }
    return subChassis;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
