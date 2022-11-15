package com.ma5951.utils.logger;

import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.subsystem.SensorSubsystem;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class Logger {
    private ArrayList<Value<Boolean>> problems;
    private ArrayList<Value<Double>> values;
    private ArrayList<Value<Double>> toSaveValues;
    private Shuffleboard board;
    private double InitTime;
    private MA_file valuesFile;
    private MA_file problemsFile;

    public Logger(SensorSubsystem[] subsystems) {
        board = new Shuffleboard("logger");
        for (int i = 0; i < subsystems.length; i++) {
            Value<Boolean>[] subsystemProblems = subsystems[i].checkForProblems();
            Value<Double>[][] subsystemValues = subsystems[i].getValuesForLogger();
            for (int j = 0;
                j < subsystemProblems.length;
                    j++) {
                        problems.add(subsystemProblems[j]);
            }
            for (int j = 0;
                j < subsystemValues.length;
                    j++) {
                        values.add(subsystemValues[j][1]);
                        toSaveValues.add(subsystemValues[j][0]);
            }
        }
        valuesFile = new MA_file("valuesFile.txt");
        problemsFile = new MA_file("problemsFile");
    }

    public void initLogger() {
        InitTime = Timer.getFPGATimestamp();
    }

    /**
     * run this function in a new thraed
     */
    public void runLogger() {
        for (int i = 0; i < problems.size(); i++) {
            Value<Boolean> problem = problems.get(i);
            if (problem.getValue()) {
                problemsFile.wirte(
                    problem.getValueName() + " at" + 
                    (Timer.getFPGATimestamp() - InitTime));
                board.addBoolean(problem.getValueName(), true);
            }
        }
        for (int i = 0; i < values.size(); i++) {
            Value<Double> value = values.get(i);
            if (toSaveValues.get(i).getValue() != 0) {
                valuesFile.wirte(
                    value.getValueName() + 
                    " is: " + value.getValue() + 
                    " at" + (Timer.getFPGATimestamp() - InitTime));
            }
        }
    }

    public void close() {
        problemsFile.close();
        valuesFile.close();
        for (int i = 0; i < problems.size(); i++) {
            board.addBoolean(problems.get(i).getValueName(), false);
        }
    }
}
