/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

public class MARunCommand extends CommandBase {
  /**
   * Creates a new RunCommand.
   */

  protected final Runnable m_toRun;
  protected final Runnable m_toRun1;

  public MARunCommand(Runnable toRun, Runnable toRun1, SubsystemBase requirements) {
    m_toRun = requireNonNullParam(toRun, "toRun", "MARunCommand");
    m_toRun1 = requireNonNullParam(toRun1, "toRun1", "MARunCommand");
    addRequirements(requirements);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_toRun.run();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_toRun1.run();
  }

}