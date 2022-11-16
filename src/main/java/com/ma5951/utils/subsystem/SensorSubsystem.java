package com.ma5951.utils.subsystem;

import com.ma5951.utils.logger.Problem;
import com.ma5951.utils.logger.Value;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SensorSubsystem extends Subsystem {
    public boolean canMove();

    /**
     * remember do not run motors in this function
     */
    public Problem[] checkForProblems();

    /**
     * remember do not run motors in this function.
     */
    public Value[] getValuesForLogger();

    public String getname();
}
