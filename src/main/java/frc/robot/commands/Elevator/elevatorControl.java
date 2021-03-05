// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.utils.JoystickContainer;
import frc.robot.utils.RobotConstants;

public class elevatorControl extends CommandBase {
  /** Creates a new elevatorControl. */
  private Elevator elevator = Elevator.getinstance();

  public elevatorControl() {
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.STICK_LEFT_Y_AXIS) > 0.5
        && elevator.getEncoderPosition() < -6000) {
      elevator.setMotorPower(ElevatorConstants.KBEST_RPM, ElevatorConstants.KELEVATOR_MOVE);
    } else if (JoystickContainer.operatingJoystick.getRawAxis(RobotConstants.STICK_LEFT_Y_AXIS) < -0.5
        && elevator.getEncoderPosition() > -162000 && elevator.getPistonValue()) {
      elevator.setMotorPower(-ElevatorConstants.KBEST_RPM, ElevatorConstants.KELEVATOR_MOVE);
    } else {
      elevator.setMotorPower(0, ElevatorConstants.KELEVATOR_MOVE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setMotorPower(0, ElevatorConstants.KELEVATOR_MOVE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
