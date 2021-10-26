// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Actuators.MAMotorControllers.MAMotorController;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Conveyor extends MASubsystem {
  private MAMotorController transportationMotor;
  private MAMotorController conveyorMotor;
  private DigitalInput transportationDigitalInput;
  private MAShuffleboard conveyorMAShuffleboard = new MAShuffleboard(ConveyorConstants.KSUBSYSTEM_NAME);
  private static Conveyor mConveyor;
  private Boolean counterLimit = true;
  private int countBall = 0;

  private Conveyor() {
    transportationMotor = new MAMotorController(MOTOR_CONTROLL.TALON,ID11);
    setMAMotorComtrolersList(transportationMotor);
    transportationDigitalInput = new DigitalInput(RobotConstants.DIO_ID0);

    conveyorMotor = new MAMotorController(MOTOR_CONTROLL.TALON,ID7);
    setMAMotorComtrolersList(conveyorMotor);

  }

  @Override
  public void periodic() {
    printValues();
    count();
  }

  /**
   * index 0 - TransportationMotor index 1 - ConveyorMotor
   */
  @Override
  public void setMotorPower(double power, int index) {
    maMotorControlers.get(index).set(power);
  }

  @Override
  public double getStatorCurrent(int index) {
    return maMotorControlers.get(index).getStatorCurrent();
  }

  @Override
  public boolean getLimitSwitchRValues() {
    return transportationDigitalInput.get();
  }

  @Override
  public void resetSensor() {
    countBall = 0;
  }

  public int count() {
    if (getLimitSwitchRValues() && counterLimit) {
      countBall++;
      counterLimit = !getLimitSwitchRValues();
    } else if (!getLimitSwitchRValues()) {
      counterLimit = !getLimitSwitchRValues();
    }
    return countBall;
  }

  public static Conveyor getinstance() {
    if (mConveyor == null) {
      mConveyor = new Conveyor();
    }
    return mConveyor;
  }

  @Override
  public void printValues() {
    conveyorMAShuffleboard.addNum("StatorCurrentTransportationMotor",
        getStatorCurrent(ConveyorConstants.KTRANSPORTATION_MOTOR));
    conveyorMAShuffleboard.addNum("CountBall", count());
    conveyorMAShuffleboard.addNum("StatorCurrentConveyorMotor", getStatorCurrent(ConveyorConstants.KCONVEYOR_MOTOR));
    conveyorMAShuffleboard.addBoolean("TransportationDigitalInputValue", getLimitSwitchRValues());
  }
}
