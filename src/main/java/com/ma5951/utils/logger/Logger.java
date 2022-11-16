package com.ma5951.utils.logger;

import com.ma5951.utils.Shuffleboard;
import com.ma5951.utils.subsystem.SensorSubsystem;

import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;

public class Logger {
    private ArrayList<Problem> problems;
    private ArrayList<Value> values;
    private Shuffleboard board;
    private double InitTime;
    private MA_file valuesFile;
    private MA_file problemsFile;

    public Logger(SensorSubsystem[] subsystems) {
        board = new Shuffleboard("logger");
        for (int i = 0; i < subsystems.length; i++) {
            Problem[] subsystemProblems = subsystems[i].checkForProblems();
            Value[] subsystemValues = subsystems[i].getValuesForLogger();
            for (int j = 0;
                j < subsystemProblems.length;
                    j++) {
                        problems.add(subsystemProblems[j]);
            }
            for (int j = 0;
                j < subsystemValues.length;
                    j++) {
                        values.add(subsystemValues[j]);
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
            Problem problem = problems.get(i);
            if (problem.getProblem()) {
                problemsFile.wirte(
                    problem.getProblemName() + " at" + 
                    (Timer.getFPGATimestamp() - InitTime));
                board.addBoolean(problem.getProblemName(), true);
            }
        }
        for (int i = 0; i < values.size(); i++) {
            Value value = values.get(i);
            if (value.toSave()) {
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
            board.addBoolean(problems.get(i).getProblemName(), false);
        }
    }
}
