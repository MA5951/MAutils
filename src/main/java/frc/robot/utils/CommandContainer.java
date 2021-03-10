// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Conveyor.Conveyor;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.PistonIntake;
import frc.robot.subsystems.Roulette.Roulette;
import frc.robot.subsystems.Shooter.MoonShooter;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Automation.Shooting;
import frc.robot.subsystems.Automation.Automation;
import frc.robot.subsystems.Balance.Balance;
import frc.robot.utils.MAComannds.MABasicMotorCommand;
import frc.robot.utils.MAComannds.MAPistonCommand;
import frc.robot.utils.MAComannds.MAPositionPIDCommand;
import frc.robot.utils.MASubsystem.MASubsystem;

/** Add your docs here. */
public class CommandContainer {
        private MASubsystem chassis = Chassis.getinstance();
        private MASubsystem elevator = Elevator.getinstance();
        private MASubsystem moonShooter = MoonShooter.getinstance();
        private MASubsystem conveyor = Conveyor.getinstance();
        private MASubsystem balance = Balance.getinstance();
        private MASubsystem roulette = Roulette.getinstance();
        private MASubsystem pistonIntake = PistonIntake.getinstance();
        private SubsystemBase automation = Automation.getinstance();

        public CommandContainer() {
                JoystickContainer.LB.whenPressed(new MAPistonCommand(pistonIntake, true));
                JoystickContainer.RB.whenPressed(new MAPistonCommand(pistonIntake, false));

                JoystickContainer.AButton.whileHeld(new MABasicMotorCommand(pistonIntake,
                                IntakeConstants.KBEST_RPM_COLLECTION, IntakeConstants.INTAKE_COLLECTION)
                                                .alongWith(new MAPistonCommand(pistonIntake, true)));
                JoystickContainer.BButton.whileHeld(new MABasicMotorCommand(pistonIntake,
                                -IntakeConstants.KBEST_RPM_COLLECTION, IntakeConstants.INTAKE_COLLECTION));

                JoystickContainer.YButton
                                .whileHeld(new StartEndCommand(() -> Automation.getinstance().conveyorControl(-1),
                                                () -> Automation.getinstance().conveyorControl(0), conveyor));
                JoystickContainer.POVDown.whileHeld(new MAPositionPIDCommand(elevator, ElevatorConstants.KDOWN_SETPOINT,
                                ElevatorConstants.KELEVATOR_MOVE));
                JoystickContainer.POVUp.whileHeld(new MAPositionPIDCommand(elevator, ElevatorConstants.KUP_SETPOINT,
                                ElevatorConstants.KELEVATOR_MOVE));

                JoystickContainer.POVRight.whenPressed(new MAPistonCommand(elevator, true));
                JoystickContainer.POVLeft.whenPressed(new MAPistonCommand(elevator, false));

                JoystickContainer.triggerR.whileActiveContinuous(new Shooting());

        }

}
