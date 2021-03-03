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
  private MAMotorControler TransportationMotor;
  private MAMotorControler ConveyorMotor;
  private DigitalInput TransportationDigitalInput;
  private DigitalInput ConveyorDigitalInput;
  private MAShuffleboard ConveyorMAShuffleboard = new MAShuffleboard(ConveyorConstants.KSUBSYSTEM_NAME);
  private static Conveyor mConveyor;

  private Conveyor() {
    TransportationMotor = new MAMotorControler(MOTOR_CONTROLL.VICTOR, IDMotor.ID13);
    setMAMotorComtrolersList(TransportationMotor);
    TransportationDigitalInput = new DigitalInput(RobotConstants.DIO_ID0);

    ConveyorMotor = new MAMotorControler(MOTOR_CONTROLL.VICTOR, IDMotor.ID14);
    setMAMotorComtrolersList(ConveyorMotor);
    ConveyorDigitalInput = new DigitalInput(RobotConstants.DIO_ID1);
  }

  @Override
  public void periodic() {
    PrintValues();
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
  public boolean getLimitSwitchFValuse() {
    return ConveyorDigitalInput.get();
  }

  @Override
  public boolean getLimitSwitchRValuse() {
    return TransportationDigitalInput.get();
  }

  public int Count() {
    return 0; // TODO
  }

  public static Conveyor getinstance(){
    if(mConveyor == null){
      mConveyor = new Conveyor();
    }
      return mConveyor;
    }
  

  @Override
  public void PrintValues() {
    ConveyorMAShuffleboard.addNum("StatorCurrentTransportationMotor",
        getStatorCurrent(ConveyorConstants.KTRANSPORTATION_MOTOR));
    ConveyorMAShuffleboard.addNum("StatorCurrentConveyorMotor", getStatorCurrent(ConveyorConstants.KCONVEYOR_MOTOR));
    ConveyorMAShuffleboard.addBoolean("ConveyorDigitalInputValue", getLimitSwitchFValuse());
    ConveyorMAShuffleboard.addBoolean("TransportationDigitalInputValue", getLimitSwitchRValuse());
  }
}
