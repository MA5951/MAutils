
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Chassis;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Autonomous.*;
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

  private void setParmater(double initTata, double initDistacn, double InitLinearSpeed) {
    this.initDistacn = initDistacn;
    this.initTata = initTata;
    this.InitLinearSpeed = InitLinearSpeed;
  }

  public MAPath() {
    chassis = Chassis.getinstance();
    addRequirements(chassis);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Path.mainPath = Path.teast;
    chassis.setidilmodeBrake(false);
    stage = 0;
    point = Path.mainPath[stage];

    if (point.getAngle() - chassis.getAngle() > 0) {
      setParmater(chassis.getAngle(), chassis.leftDistance(),
          MACalculations.fromRPMToLinearSpeed(chassis.leftRPM(), ChassisConstants.KCHASSIS_GEAR));
    } else if (point.getAngle() - chassis.getAngle() < 0) {
      setParmater(chassis.getAngle(), chassis.rigthDistance(),
          MACalculations.fromRPMToLinearSpeed(chassis.rightRPM(), ChassisConstants.KCHASSIS_GEAR));
    } else {
      setParmater(chassis.getAngle(), (chassis.rigthDistance() + chassis.leftDistance()) / 2, MACalculations
          .fromRPMToLinearSpeed((chassis.rightRPM() + chassis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR));
    }

    chassis.setpoint(point, InitLinearSpeed, initDistacn, initTata);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.pathMotorOutPut();
    if (stage + 1 != Path.mainPath.length) {
      if (chassis.atPoint(point)) {
        stage++;
        point = Path.mainPath[stage];
        setParmater(chassis.getAngle(), 0, MACalculations
            .fromRPMToLinearSpeed((chassis.rightRPM() + chassis.leftRPM()) / 2, ChassisConstants.KCHASSIS_GEAR));
        chassis.reserEncoder();
      }
    } else {
      stage++;
    }
    chassis.setpoint(point, InitLinearSpeed, initDistacn, initTata);

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

    if (stage == Path.mainPath.length) {
      return true;
    }
    return false;
  }
}
