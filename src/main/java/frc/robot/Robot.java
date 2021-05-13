/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Chassis.MAPath;
import frc.robot.commands.Elevator.elevatorControl;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.utils.CommandContainer;
import frc.robot.utils.MADriverStation;
import frc.robot.utils.limelight;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    CommandContainer commandContainer = new CommandContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    limelight.periodic();
    MADriverStation.getinstance().periodic();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Chassis.getinstance().setidilmodeBrake(true);
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link ButtonContainer} class.
   */
  @Override
  public void autonomousInit() {
    MAPath.pathnum = 0;
    Chassis.getinstance().resetValue();
    m_autonomousCommand = new MAPath(); // Autonomous.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    Chassis.getinstance().setidilmodeBrake(false);
    Chassis.getinstance().resetValue();

    // CommandScheduler.getInstance().setDefaultCommand(Chassis.getinstance(), new
    // TankDrive());
    // CommandScheduler.getInstance().setDefaultCommand(Balance.getinstance(), new
    // BalanceContorl());
    CommandScheduler.getInstance().setDefaultCommand(Elevator.getinstance(), new elevatorControl());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // Chassis.getinstance().pathMotorOutPut();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
