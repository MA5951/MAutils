// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.utils.MASubsystem.MASubsystem;

/** Add your docs here. */
public class ShooterContainer {
    private static MASubsystem linearShooter = LinearShooter.getinstance();
    private static MASubsystem moonShooter = MoonShooter.getinstance();
    public static void config() {

    }
}
