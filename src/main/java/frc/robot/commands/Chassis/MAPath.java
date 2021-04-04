
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
  private double initTata = 0;
  private double InitLinearSpeed = 0;
  private Point point;
  private autonomousState lastStae = autonomousState.STRAIGHT_LINE;

  private void setParmater(double initTata, double initDistacn, double InitLinearSpeed) {
    this.initDistacn = initDistacn;
    this.initTata = initTata;
    this.InitLinearSpeed = InitLinearSpeed;
  }

  private double getInitDistace() {
    if (chassis.state == autonomousState.STRAIGHT_LINE || chassis.state == autonomousState.TURN_IN_PLACE) {
      return (chassis.leftDistance() + chassis.rigthDistance() / 2);
    } else if (chassis.state == autonomousState.RIGHT && lastStae != autonomousState.LEFT) {
      return chassis.leftDistance();
    } else if (chassis.state == autonomousState.LEFT && lastStae != autonomousState.RIGHT) {
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
    Path.mainPath = Path.teast;
    stage = 0;
    point = Path.mainPath[stage];
    chassis.setAutonomousState(point);
    Autonomous.setLastDistance(point.getDistance());
    chassis.setCircelRaduis();

    if (chassis.state == autonomousState.RIGHT) {
      setParmater(chassis.getAngle(), getInitDistace(),
          MACalculations.fromRPMToLinearSpeed(chassis.leftRPM(), ChassisConstants.KCHASSIS_GEAR));
    } else if (chassis.state == autonomousState.LEFT) {
      setParmater(chassis.getAngle(), getInitDistace(),
          MACalculations.fromRPMToLinearSpeed(chassis.rightRPM(), ChassisConstants.KCHASSIS_GEAR));
    } else {
      setParmater(chassis.getAngle(), getInitDistace(), MACalculations
          .fromRPMToLinearSpeed((chassis.rightRPM() + chassis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR));
    }
    lastStae = chassis.state;
    chassis.setpoint(point, InitLinearSpeed, initDistacn, initTata);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (stage + 1 != Path.mainPath.length) {
      if (chassis.atPoint(point)) {
        setParmater(chassis.getAngle(), getInitDistace(), MACalculations
            .fromRPMToLinearSpeed((chassis.rightRPM() + chassis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR));
        stage++;
        point = Path.mainPath[stage];
        chassis.setAutonomousState(point);
        lastStae = chassis.state;
        Autonomous.setLastDistance(point.getDistance());
        chassis.setCircelRaduis();
        chassis.reserValueForAutonomous();
      }
    }
    

    chassis.setpoint(point, InitLinearSpeed, initDistacn, initTata);
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
