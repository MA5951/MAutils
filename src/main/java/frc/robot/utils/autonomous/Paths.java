// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autonomous;

import frc.robot.utils.autonomous.Path.PathBuilder;

/** Add your docs here. */
public class Paths {
    public static PathBuilder builder = new Path.PathBuilder();
    public static Path testPath = builder
                                        .setSpacing(0.1)
                                        .setK(1)
                                        .setMaxVelocity(1)
                                        .setMaxAcceleration(0.2)
                                        .setLookaheadDistance(0.1)
                                        .setMaxRate(0.05)
                                        .addPoint(-1, 0, true)
                                        .build();

}
