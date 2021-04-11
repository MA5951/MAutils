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

      if (velocity > point.getEndVelocity())
        return velocity = velocity - (point.getAcceleration() * RobotConstants.KDELTA_TIME);
      else {
        return point.getEndVelocity();
      }
    }
  }

  public static double theOtherVelocitySetPoint(double V, double theta) {
    double sight = Math.abs(V) / V;
    double thetaToRadians = Math.toRadians(theta);
    if (V == 0) {
      return 0;
    }
    return V - ((2 * ChassisConstants.KchassisLength / Math.tan(thetaToRadians)) * sight);
  }

  public static double thetaFromDistance(double distacne, Point point) {

    if (distacne == 0) {
      return 10 * point.getAngle() / Math.abs(point.getAngle());
    }
    
    return Math.toDegrees(distacne / point.getCircelRauis());
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