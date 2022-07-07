package com.ma5951.utils.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface PistonSubsystem extends Subsystem {
    public void open();

    public void close();

    public boolean isOpen();

    public void off();
}
