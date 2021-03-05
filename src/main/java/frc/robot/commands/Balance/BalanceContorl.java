// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Balance.Balance;
import frc.robot.subsystems.Balance.BalanceConstants;
import frc.robot.utils.JoystickContainer;
import frc.robot.utils.RobotConstants;

public class BalanceContorl extends CommandBase {

  private Balance balance = Balance.getinstance();

  public BalanceContorl() {
    addRequirements(balance);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    balance.setMotorPower(JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.STICK_RIGHT_X_AXIS),
        BalanceConstants.MOTOR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    balance.setMotorPower(0, BalanceConstants.MOTOR);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
