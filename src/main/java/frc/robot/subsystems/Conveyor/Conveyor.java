// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MASubsystem;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Conveyor extends SubsystemBase implements MASubsystem {
  private MAMotorControler TransportationMotor;
  private MAMotorControler ConveyorMotor;
  private DigitalInput TransportationDigitalInput;
  private DigitalInput ConveyorDigitalInput;
  private MAShuffleboard ConveyorMAShuffleboard = new MAShuffleboard(ConveyorConstants.KSUBSYSTEM_NAME);

  public Conveyor() {
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
  public Runnable setMotorPower(double power, int Indax) {
    return () -> maMotorControlers.get(Indax).set(power);
  }

  public double getStatorCurrent(int Motor) {
    return maMotorControlers.get(Motor).getStatorCurrent();
  }

  public boolean getConveyorDigitalInputValue() {
    return ConveyorDigitalInput.get();
  }

  public boolean getTransportationDigitalInputValue() {
    return TransportationDigitalInput.get();
  }

  public int Count() {
    return 0; // TODO
  }

  @Override
  public void PrintValues() {
    ConveyorMAShuffleboard.addNum("StatorCurrentTransportationMotor",
        getStatorCurrent(ConveyorConstants.KTRANSPORTATION_MOTOR));
    ConveyorMAShuffleboard.addNum("StatorCurrentConveyorMotor", getStatorCurrent(ConveyorConstants.KCONVEYOR_MOTOR));
    ConveyorMAShuffleboard.addBoolean("ConveyorDigitalInputValue", getConveyorDigitalInputValue());
    ConveyorMAShuffleboard.addBoolean("TransportationDigitalInputValue", getTransportationDigitalInputValue());
  }
}
