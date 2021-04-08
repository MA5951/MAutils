
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Autonomous.*;
import frc.robot.utils.Autonomous.Autonomous.autonomousState;
import frc.robot.utils.Calculation.MACalculations;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Chassis.ChassisConstants;

public class MAPath extends CommandBase {
  /**
   * Creates a new MAPath.
   */

  Chassis chassis;
  public static int stage = 0;
  public static int pathnum = 0;
  private double initDistacn = 0;
  private double initTheta = 0;
  private double InitLinearSpeed = 0;
  private Point point;
  private autonomousState lastStae = autonomousState.STRAIGHT_LINE;

  private void setParmater(double initTheta, double initDistacn, double InitLinearSpeed) {
    this.initDistacn = initDistacn;
    this.initTheta = initTheta;
    this.InitLinearSpeed = InitLinearSpeed;
  }

  private double getInitDistace() {
    if (Autonomous.state == autonomousState.STRAIGHT_LINE || Autonomous.state == autonomousState.TURN_IN_PLACE) {
      return (chassis.leftDistance() + chassis.rigthDistance() / 2);
    } else if (Autonomous.state == autonomousState.RIGHT && lastStae != autonomousState.LEFT) {
      return chassis.leftDistance();
    } else if (Autonomous.state == autonomousState.LEFT && lastStae != autonomousState.RIGHT) {
      return chassis.rigthDistance();
    }
    return (chassis.leftDistance() + chassis.rigthDistance() / 2);
  }

  public MAPath() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Autonomous.reserValueForAutonomous();
    Path.mainPath = Path.teast;
    stage = 0;
    point = Path.mainPath[stage];
    Autonomous.setAutonomousState(point);
    Autonomous.setLastDistance(point.getDistance());
    Autonomous.setCircelRaduis(point);

    if (Autonomous.state == autonomousState.RIGHT) {
      setParmater(chassis.getAngle(), getInitDistace(),
          MACalculations.fromRPMToLinearSpeed(chassis.leftRPM(), ChassisConstants.KCHASSIS_GEAR));
    } else if (Autonomous.state == autonomousState.LEFT) {
      setParmater(chassis.getAngle(), getInitDistace(),
          MACalculations.fromRPMToLinearSpeed(chassis.rightRPM(), ChassisConstants.KCHASSIS_GEAR));
    } else {
      setParmater(chassis.getAngle(), getInitDistace(), MACalculations
          .fromRPMToLinearSpeed((chassis.rightRPM() + chassis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR));
    }
    lastStae = Autonomous.state;
    Autonomous.calculatTimeAndDistanceToAutonomous(InitLinearSpeed, point);
    chassis.setpoint(point, InitLinearSpeed, initDistacn, initTheta);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (stage + 1 != Path.mainPath.length) {
      if (Autonomous.atPoint(point)) {
        setParmater(chassis.getAngle(), getInitDistace(), MACalculations
            .fromRPMToLinearSpeed((chassis.rightRPM() + chassis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR));
        stage++;
        point = Path.mainPath[stage];
        Autonomous.setAutonomousState(point);
        lastStae = Autonomous.state;
        Autonomous.setLastDistance(point.getDistance());
        Autonomous.setCircelRaduis(point);
        Autonomous.reserValueForAutonomous();
      }
    }

    chassis.setpoint(point, InitLinearSpeed, initDistacn, initTheta);
    Autonomous.calculatTimeAndDistanceToAutonomous(InitLinearSpeed, point);
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

    if (stage + 1 >= Path.mainPath.length && Autonomous.atPoint(Path.mainPath[Path.mainPath.length - 1])) {
      return true;
    }
    return false;
  }
}
