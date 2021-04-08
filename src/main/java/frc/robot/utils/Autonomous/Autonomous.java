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
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Chassis.ChassisConstants;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Calculation.MACalculations;

public class Autonomous {

  public static final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static String selected;
  private static double lastDistance = 0;

  public static enum autonomousState {
    RIGHT, LEFT, STRAIGHT_LINE, TURN_IN_PLACE
  }

  public static autonomousState state = autonomousState.STRAIGHT_LINE;

  public static double velocity = 0;
  public static double timeInPoint = 0;
  private static double rightCircelRadius = 0;
  private static double leftCircelRadius = 0;

  private static double timeInMaxSpeed = 0;
  private static double accelerationTimeToSetPoint = 0;
  private static double accelerationTimeToMaxSpeed = 0;
  private static double distacnePassInAccelerationMove = 0;
  private static double distacnePassInAccelerationToSetPoint = 0;
  private static double deltaTheta = 0;
  private static Chassis chasis = Chassis.getinstance();

  public static double getRightCircelRadius() {
    return rightCircelRadius;
  }

  public static double getLeftCircelRadius() {
    return leftCircelRadius;
  }

  public static double getDeltaTheta() {
    return deltaTheta;
  }

  public static void setLastDistance(double LastDistance) {
    lastDistance = LastDistance;
  }

  public static autonomousState getState(double tata, double initTheta, double distacne) {
    double sight = distacne / Math.abs(distacne);
    if (distacne == lastDistance) {
      deltaTheta = Math.toRadians(tata - initTheta);
      return autonomousState.TURN_IN_PLACE;
    } else if ((tata - initTheta) * sight > 0) {
      deltaTheta = Math.toRadians(tata - initTheta);
      return autonomousState.RIGHT;
    } else if ((tata - initTheta) * sight < 0) {
      deltaTheta = Math.toRadians(tata - initTheta);
      return autonomousState.LEFT;
    } else if (tata == initTheta) {
      deltaTheta = 0;
      return autonomousState.STRAIGHT_LINE;
    }
    deltaTheta = 0;
    return autonomousState.STRAIGHT_LINE;
  }

  public static boolean atPoint(Point point) {
    if (state == autonomousState.RIGHT) {
      return point.getAngle() == chasis.getAngle() && Math.abs(point.getArcDistance() - chasis.leftDistance()) < 0.2
          && Math.abs(point.getEndVelocity()
              - MACalculations.fromRPMToLinearSpeed(chasis.leftRPM(), ChassisConstants.KCHASSIS_GEAR)) < 0.1;
    } else if (state == autonomousState.LEFT) {
      return point.getAngle() == chasis.getAngle() && Math.abs(point.getArcDistance() - chasis.rigthDistance()) < 0.2
          && Math.abs(point.getEndVelocity()
              - MACalculations.fromRPMToLinearSpeed(chasis.rightRPM(), ChassisConstants.KCHASSIS_GEAR)) < 0.1;
    } else {
      return point.getAngle() == chasis.getAngle()
          && Math.abs(point.getDistance() - (chasis.rigthDistance() + chasis.leftDistance()) / 2) < 0.2
          && Math.abs(point.getEndVelocity() - MACalculations
              .fromRPMToLinearSpeed((chasis.rightRPM() + chasis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR)) < 0.1;
    }
  }

  public static void calculatTimeAndDistanceToAutonomous(double LinearSpeed, Point point) {
    timeInPoint = timeInPoint + RobotConstants.KDELTA_TIME;
    accelerationTimeToSetPoint = Math.abs((point.getMaxVelocity() - point.getEndVelocity()) / point.getAcceleration());

    accelerationTimeToMaxSpeed = Math.abs((point.getMaxVelocity() - LinearSpeed) / point.getAcceleration());

    distacnePassInAccelerationMove = LinearSpeed * accelerationTimeToMaxSpeed
        + (point.getAcceleration() / 2) * Math.pow(accelerationTimeToMaxSpeed, 2);

    distacnePassInAccelerationToSetPoint = point.getMaxVelocity() * accelerationTimeToSetPoint
        - (point.getAcceleration() / 2) * Math.pow(accelerationTimeToSetPoint, 2);

    if (state == autonomousState.RIGHT || state == autonomousState.LEFT) {
      timeInMaxSpeed = Math
          .abs((point.getArcDistance() - (distacnePassInAccelerationToSetPoint + distacnePassInAccelerationMove))
              / point.getMaxVelocity());
    } else {
      timeInMaxSpeed = Math
          .abs((point.getDistance() - (distacnePassInAccelerationToSetPoint + distacnePassInAccelerationMove))
              / point.getMaxVelocity());
    }
  }

  public static double accelerationVelocitySetPoint(double LinearSpeed, Point point) {
    if (timeInPoint < accelerationTimeToMaxSpeed && Math
        .abs((velocity + point.getAcceleration() * RobotConstants.KDELTA_TIME)) < Math.abs(point.getMaxVelocity())) {
      velocity = velocity + LinearSpeed + (point.getAcceleration() * RobotConstants.KDELTA_TIME);
      return velocity;
    } else if (timeInPoint + RobotConstants.KDELTA_TIME < timeInMaxSpeed + accelerationTimeToMaxSpeed) {
      return point.getMaxVelocity();
    } else {
      if (Math.abs(velocity) <= Math.abs(point.getEndVelocity()))
        return point.getEndVelocity();
      else {
        velocity = velocity - (point.getAcceleration() * RobotConstants.KDELTA_TIME);
        return velocity;
      }
    }
  }

  public static double theOtherVelocitySetPoint(double V, double theta) {
    double sight = Math.abs(V) / V;
    double thetaToRadians = Math.toRadians(theta);
    if (state == autonomousState.RIGHT || state == autonomousState.LEFT)
      return V - ((2 * ChassisConstants.KchassisLength / Math.tan(thetaToRadians)) * sight);

    if (state == autonomousState.TURN_IN_PLACE) {
      return -V;
    }

    return V;
  }

  public static double thetaFromDistance(double maxDistacn, double maxtheta, double initDistacn, double inittheta) {
    double sight = Math.abs(maxtheta) / maxtheta;
    double a = maxDistacn * Math.cos(maxtheta);
    if (Math.abs(inittheta) + 10 > Math.abs(chasis.getAngle())) {
      return inittheta + 10 * sight;
    }

    if (Math.abs(chasis.getAngle()) + 10 >= Math.abs(maxtheta)) {
      return maxtheta;
    }

    if (state == autonomousState.RIGHT) {
      double H = chasis.rigthDistance() - initDistacn;
      return Math.toDegrees(Math.atan(H / a));
    } else if (state == autonomousState.LEFT) {
      double H = chasis.leftDistance() - initDistacn;
      return Math.toDegrees(Math.atan(H / a)) * -1;
    }

    return 0;
  }

  public static void setAutonomousState(Point point) {
    state = Autonomous.getState(point.getAngle(), chasis.getAngle(), point.getDistance()); // TODO
  }

  public static void reserValueForAutonomous() {
    velocity = 0;
    timeInPoint = 0;
    leftCircelRadius = 0;
    rightCircelRadius = 0;
    accelerationTimeToSetPoint = 0;
    accelerationTimeToMaxSpeed = 0;
    distacnePassInAccelerationMove = 0;
    distacnePassInAccelerationToSetPoint = 0;
    timeInPoint = 0;
  }

  public static void setCircelRaduis(Point point) {
    if (state == autonomousState.RIGHT) {
      leftCircelRadius = Math.sqrt((Math.pow(point.getDistance(), 2) / (2 - 2 * Math.cos(deltaTheta))));
      rightCircelRadius = leftCircelRadius + ChassisConstants.KchassisLength;
    } else if (state == autonomousState.LEFT) {
      rightCircelRadius = Math.sqrt((Math.pow(point.getDistance(), 2) / (2 - 2 * Math.cos(deltaTheta))));
      leftCircelRadius = rightCircelRadius + ChassisConstants.KchassisLength;
    } else if (state == autonomousState.TURN_IN_PLACE) {
      leftCircelRadius = Math.sqrt((Math.pow(point.getDistance(), 2) / (2 - 2 * Math.cos(deltaTheta))));
      rightCircelRadius = leftCircelRadius;
    } else {
      leftCircelRadius = 0;
      rightCircelRadius = leftCircelRadius;
    }
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