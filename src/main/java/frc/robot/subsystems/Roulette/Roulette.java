// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Roulette;

import frc.robot.utils.MASubsystem.MASubsystem;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Actuators.MAPiston;
import frc.robot.utils.Actuators.MAMotorControlrs.MAMotorControler;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

public class Roulette extends MASubsystem {
  private MAPiston piston;
  private MAMotorControler rouletteMotor;
  private ColorSensorV3 colorSensorV3;
  private static Roulette m_Roulette;
  private MAShuffleboard rouletteShuffleboard = new MAShuffleboard(RouletteConstants.KSUBSYSTEM_NAME);

  private Roulette() {
    rouletteMotor = new MAMotorControler(MOTOR_CONTROLL.VICTOR, IDMotor.ID10);
    piston = new MAPiston(RobotConstants.P_ID6);
    colorSensorV3 = new ColorSensorV3(Port.kOnboard);
    setMAMotorComtrolersList(rouletteMotor);

  }

  @Override
  public void periodic() {
    printValues();
  }

  /**
   * Motor - 0
   */
  @Override
  public void setMotorPower(double power, int Indax) {
    maMotorControlers.get(Indax).set(power);
  }

  @Override
  public double getStatorCurrent() {
    return maMotorControlers.get(0).getStatorCurrent();
  }

  @Override
  public void togglePiston() {
    piston.toggle();
  }

  @Override
  public double getOutput() {
    return maMotorControlers.get(0).getOutput();
  }

  @Override
  public void printValues() {
    rouletteShuffleboard.addNum("SingleMotorgetStatorCurrent",
        maMotorControlers.get(RouletteConstants.MOTOR).getStatorCurrent());
    rouletteShuffleboard.addBoolean("getPistonMod", piston.get());
    rouletteShuffleboard.addNum("SingalMotorgetOutput", maMotorControlers.get(RouletteConstants.MOTOR).getOutput());
  }

  public static Roulette getinstance() {
    if (m_Roulette == null) {
      m_Roulette = new Roulette();
    }
    return m_Roulette;
  }
}