// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MASubsystem;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MASubsystem.Intrfaces.MASubsystemInterface;


/** Add your docs here. */
public class MASubsystem extends SubsystemBase implements MASubsystemInterface {

  protected ArrayList<MAMotorControler> maMotorControlers = new ArrayList<MAMotorControler>();

  protected SubConstants constants;

  public enum ENCODER {
    No_Encoder, Encoder, Alternate_Encoder
  }

  public enum MOTOR_CONTROLL {
    TALON, VICTOR, SPARKMAXBrushless, SPARKMAXBrushled
  }

  public enum IDMotor {
    ID1, ID2, ID3, ID4, ID5, ID6, ID7, ID8, ID9, ID10, ID11, ID12, ID13, ID14, ID15, ID16
  }

  public void setMAMotorComtrolersList(MAMotorControler motor) {
    maMotorControlers.add(motor);
  }

  public void addFollowMotorToMaster(MAMotorControler master, IDMotor id) {
    MAMotorControler Motor = new MAMotorControler(master.getMotorControllType(), id);
    Motor.follow(master);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void setMotorPower(double power, int Indax) {

  }

  @Override
  public void printValues() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setMotorVoltage(double voltage, int indax) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getEncoderPosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean getLimitSwitchFValuse() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public boolean getLimitSwitchRValuse() {
    // TODO Auto-generated method stub
    return false;
  }





  @Override
  public double getStatorCurrent() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getStatorCurrent(int indax) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getEncdoerRPM() {
    // TODO Auto-generated method stub
    return 0;
  }



  @Override
  public void resetSensor() {
    // TODO Auto-generated method stub

  }

  @Override
  public double getOutput() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getEncoderPosition(int indax) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getEncdoerRPM(int indax) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean getLimitSwitchFValuse(int indax) {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public boolean getLimitSwitchRValuse(int indax) {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void setPiston(boolean on) {
    // TODO Auto-generated method stub

  }

  @Override
  public void togglePiston() {
    // TODO Auto-generated method stub

  }

  @Override
  public boolean getPistonValue() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double calculatePIDOutput(double setPoint) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double calculatePIDOutput() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void setSetPoint(double setPoint) {
    // TODO Auto-generated method stub

  }

  @Override
  public double resetPID() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean isPIDAtTarget() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double getSetpointPID() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getPositionError() {
    // TODO Auto-generated method stub
    return 0;
  }

}
