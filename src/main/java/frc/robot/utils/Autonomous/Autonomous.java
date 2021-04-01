/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Chassis.MAPath;

public class Autonomous {

  public static final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static String selected;

  public Autonomous() {

  }

  public static void setAutonomousCommand() {
    SmartDashboard.putData("Chooser", m_chooser);
    m_chooser.addOption("Enemy Roullete", "EnemyRoullete");
    m_chooser.addOption("Roullete Path", "RoulletePath");
    m_chooser.addOption("Roullete Path 1", "RoulletePath1");
    m_chooser.addOption("Shoot And Drive", "shootanddrive");
    m_chooser.addOption("Standart 1", "standart1");
    selected = m_chooser.getSelected();
  }

  public static Command getAutonomousCommand() {
    if (m_chooser.getSelected() == "EnemyRoullete") {
      return new MAPath();// TODO
    } else if (m_chooser.getSelected() == "RoulletePath") {
      return new MAPath();// TODO
    } else if (m_chooser.getSelected() == "RoulletePath1") {
      return new MAPath();// TODO
    } else if (m_chooser.getSelected() == "shootanddrive") {
      return new MAPath();// TODO
    } else if (m_chooser.getSelected() == "standart1") {
      return new MAPath(); // TODO
    } else {
      return null;
    }
  }

}