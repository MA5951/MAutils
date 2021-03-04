// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Chassis.Chassis;
import frc.robot.subsystems.Conveyor.Conveyor;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.MotorIntake;
import frc.robot.subsystems.Intake.PistonIntake;
import frc.robot.subsystems.Shooter.LinearShooter;
import frc.robot.subsystems.Shooter.MoonShooter;
import frc.robot.subsystems.SingleMotor.SingleMotor;
import frc.robot.utils.MASubsystem.MASubsystem;

/** Add your docs here. */
public class CommandContainer {
    private MASubsystem chassis = Chassis.getinstance();
    private MASubsystem elevator = Elevator.getinstance();
    private MASubsystem arm = Arm.getinstance();
    private MASubsystem moonShooter = MoonShooter.getinstance();
    private MASubsystem linearShooter = LinearShooter.getinstance();
    private MASubsystem conveyor = Conveyor.getinstance();
    private MASubsystem singleMotor = SingleMotor.getinstance();
    private MASubsystem motorIntake = MotorIntake.getinstance();
    private MASubsystem pistonIntake = PistonIntake.getinstance();

    public CommandContainer() {

    }

}
