package com.ma5951.utils.subsystem;

import com.ma5951.utils.logger.Value;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SensorSubsystem extends Subsystem {
    public boolean canMove();

    /**
     * remember do not run motors in this function
     */
    public Value<Boolean>[] checkForProblems();

    /**
     * remember do not run motors in this function.
     * the raw side is when to save the value (0 means false); 
     */
    public Value<Double>[][] getValuesForLogger();

    public String getname();
}
