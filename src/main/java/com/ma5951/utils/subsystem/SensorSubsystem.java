package com.ma5951.utils.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SensorSubsystem extends Subsystem {
    public boolean canMove();

    public void reset();
}
