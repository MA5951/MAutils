// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MASubsystem;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Actuators.MAMotorControllers.MAMotorController;
import frc.robot.utils.MASubsystem.Intrfaces.MASubsystemInterface;

/** Add your docs here. */
public class MASubsystem extends SubsystemBase implements MASubsystemInterface {

  protected ArrayList<MAMotorController> maMotorControlers = new ArrayList<MAMotorController>();

  protected SubConstants constants;

  public enum ENCODER {
    No_Encoder, Encoder, Alternate_Encoder
  }

  public enum MOTOR_CONTROLL {
    TALON, VICTOR, SPARKMAXBrushless, SPARKMAXBrushled
  }

  protected static int ID1 = 1, ID2 = 2, ID3 = 3, ID4 = 4, ID5 = 5, ID6 = 6, ID7 = 7, ID8 = 8, ID9 = 9, ID10 = 10,
      ID11 = 11, ID12 = 12, ID13 = 13, ID14 = 14, ID15 = 15, ID16 = 16;

  public void setMAMotorComtrolersList(MAMotorController motor) {
    maMotorControlers.add(motor);
  }

  public void addFollowMotorToMaster(MAMotorController master, int id) {
    MAMotorController Motor = new MAMotorController(master.getMotorControllType(), id);
    Motor.follow(master);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void setMotorPower(double power, int Index) {

  }

  @Override
  public void printValues() {
    // TODO Auto-generated method stub

  }

  @Override
  public void setMotorVoltage(double voltage, int index) {
    // TODO Auto-generated method stub

  }

  @Override
  public double getEncoderPosition() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean getLimitSwitchFValues() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public boolean getLimitSwitchRValues() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public double getStatorCurrent() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getStatorCurrent(int index) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getEncoderRPM() {
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
  public double getEncoderPosition(int index) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public double getEncoderRPM(int index) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean getLimitSwitchFValues(int index) {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public boolean getLimitSwitchRValues(int index) {
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
