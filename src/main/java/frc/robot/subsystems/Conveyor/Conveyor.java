// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.MASubsystem.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Conveyor extends MASubsystem {
  private MAMotorControler transportationMotor;
  private MAMotorControler conveyorMotor;
  private DigitalInput transportationDigitalInput;
  private MAShuffleboard conveyorMAShuffleboard = new MAShuffleboard(ConveyorConstants.KSUBSYSTEM_NAME);
  private static Conveyor mConveyor;
  private Boolean counterLimit = true;
  private int countBall = 0;

  private Conveyor() {
    transportationMotor = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID11);
    setMAMotorComtrolersList(transportationMotor);
    transportationDigitalInput = new DigitalInput(RobotConstants.DIO_ID0);

    conveyorMotor = new MAMotorControler(MOTOR_CONTROLL.TALON, IDMotor.ID7);
    setMAMotorComtrolersList(conveyorMotor);

  }

  @Override
  public void periodic() {
    printValues();
    count();
  }

  /**
   * indax 0 - TransportationMotor indax 1 - ConveyorMotor
   */
  @Override
  public void setMotorPower(double power, int indax) {
    maMotorControlers.get(indax).set(power);
  }

  @Override
  public double getStatorCurrent(int indax) {
    return maMotorControlers.get(indax).getStatorCurrent();
  }

  @Override
  public boolean getLimitSwitchRValuse() {
    return transportationDigitalInput.get();
  }

  @Override
  public void resetSensor() {
    countBall = 0;
  }

  public int count() {
    if (getLimitSwitchRValuse() && counterLimit) {
      countBall++;
      counterLimit = !getLimitSwitchRValuse();
    } else if (!getLimitSwitchRValuse()) {
      counterLimit = !getLimitSwitchRValuse();
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
    conveyorMAShuffleboard.addBoolean("TransportationDigitalInputValue", getLimitSwitchRValuse());
  }
}
