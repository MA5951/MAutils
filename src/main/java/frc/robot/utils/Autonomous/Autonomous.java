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
import frc.robot.subsystems.Chassis.ChassisConstants;
import frc.robot.utils.RobotConstants;

public class Autonomous {

  public static final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static String selected;

  private static double velocity = 0;

  public static void setVelocity(double Velocity) {
    velocity = Velocity;
  }

  public static double getVelocity() {
    return velocity;
  }

  public static double accelerationVelocitySetPoint(Point point) {
    if (point.getTimeInPoint() < point.getAccelerationTimeToMaxSpeed() && Math
        .abs((velocity + point.getAcceleration() * RobotConstants.KDELTA_TIME)) < Math.abs(point.getMaxVelocity())) {
      velocity = velocity + point.getLastPoint().getEndVelocity()
          + (point.getAcceleration() * RobotConstants.KDELTA_TIME);

      return velocity;
    } else if (point.getTimeInPoint() + RobotConstants.KDELTA_TIME < point.getTimeInMaxSpeed()
        + point.getAccelerationTimeToMaxSpeed()) {

      return point.getMaxVelocity();
    } else {

      if (point.getTimeInPoint() + RobotConstants.KDELTA_TIME < point.getAccelerationTimeToSetPoint()
          + point.getTimeInMaxSpeed() + point.getAccelerationTimeToMaxSpeed()
          || Math.abs(velocity) < Math.abs(point.getEndVelocity()))
        return point.getEndVelocity();
      else {
        return velocity = velocity - (point.getAcceleration() * RobotConstants.KDELTA_TIME);
      }
    }
  }

  public static double theOtherVelocitySetPoint(double V, Point point) {
    return ((point.getCircelRadius() - ChassisConstants.KchassisLength) / point.getCircelRadius()) * V;

  }

  public static double thetaFromDistance(double distance, Point point) {
    double sight = Math.abs(point.getAngle()) / point.getAngle();

    if (Math.abs(point.getArcDistance() - distance) < 0.2
        || Math.abs(Math.toDegrees(distance / point.getCircelRadius()) * sight) > Math.abs(point.getAngle())) {

      return point.getAngle();
    }

    return Math.toDegrees(distance / point.getCircelRadius()) * sight;
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