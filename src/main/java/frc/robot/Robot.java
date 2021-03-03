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
import frc.robot.commands.Chassis.TankDrive;
import frc.robot.subsystems.Automation.Automation;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.utils.Autonomous.Autonomous;
import frc.robot.utils.MADriverStation;
import frc.robot.utils.limelight;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private TankDrive tankDrive = new TankDrive();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    MADriverStation.getinstance();
    Autonomous.setAutonomousCommand();
    Chassis.getinstance();
    Automation.getinstance();
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
    Chassis.getinstance().setidilmodeBrake(false);
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
    m_autonomousCommand = Autonomous.getAutonomousCommand();

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
    Chassis.getinstance().rampRate(0);
    Chassis.getinstance().setidilmodeBrake(false);
    CommandScheduler.getInstance().setDefaultCommand(Chassis.getinstance(), tankDrive);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

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
