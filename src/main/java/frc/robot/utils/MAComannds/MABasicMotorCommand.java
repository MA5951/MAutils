// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAComannds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.MASubsystem.MASubsystem;

public class MABasicMotorCommand extends CommandBase {
  /** Creates a new MABasicMotorCommand. */
  private MASubsystem currentSubsystem;
  private double power;
  private double voltage;
  private int indax;
  private boolean mod =false;

  public MABasicMotorCommand(MASubsystem currentSubsystem, double power, int indax) {
    this.power = power;
    this.indax = indax;
    this.currentSubsystem = currentSubsystem;
    addRequirements(this.currentSubsystem);
  }

  public MABasicMotorCommand(MASubsystem currentSubsystem, double voltage, int indax, boolean mod) {
    this.voltage = voltage;
    this.indax = indax;
    this.mod = mod;
    this.currentSubsystem = currentSubsystem;
    addRequirements(this.currentSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!mod){
      this.currentSubsystem.setMotorPower(power, indax);
    }else{
      this.currentSubsystem.setMotorVoltage(voltage, indax);
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.currentSubsystem.setMotorPower(0, indax);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
