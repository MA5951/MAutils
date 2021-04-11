
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.RobotConstants;
import frc.robot.utils.Autonomous.*;
import frc.robot.subsystems.Chassis.Chassis;

public class MAPath extends CommandBase {
  /**
   * Creates a new MAPath.
   */

  private Chassis chassis;
  public static int stage = 0;
  public static int pathnum = 0;
  public static Point point = new Point(0, 0);
  private double timeInPoint = 0;

  public MAPath() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Path.mainPath = Path.teast;
    stage = 0;
    timeInPoint = RobotConstants.KDELTA_TIME;
    point = Path.mainPath[stage];
    point.setLastPoint(new Point(0, 0, 4, 0));
    point.setTimeInPoint(timeInPoint);
    point.setState();
    point.setCircelRaduis();
    point.calculatTimeAndDistanceToAutonomous();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (stage + 1 != Path.mainPath.length) {
      if (chassis.atPoint(point)) {
        Point lasPoint = point;
        Autonomous.setVelocity(0);
        timeInPoint = RobotConstants.KDELTA_TIME;
        stage++;
        point = Path.mainPath[stage];
        point.setTimeInPoint(timeInPoint);
        point.setLastPoint(lasPoint);
        point.setState();
        point.setCircelRaduis();
        point.calculatTimeAndDistanceToAutonomous();

      }
    }

    timeInPoint = timeInPoint + RobotConstants.KDELTA_TIME;
    point.setTimeInPoint(timeInPoint);
    chassis.setpoint(point);
    chassis.pathMotorOutPut();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      chassis.leftcontrol(0);
      chassis.rightcontrol(0);
      chassis.setidilmodeBrake(true);
    } else {

      pathnum++;
      chassis.leftcontrol(0);
      chassis.rightcontrol(0);
      chassis.setidilmodeBrake(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (stage + 1 >= Path.mainPath.length && chassis.atPoint(Path.mainPath[Path.mainPath.length - 1])) {
      return true;
    }
    return false;
  }
}
