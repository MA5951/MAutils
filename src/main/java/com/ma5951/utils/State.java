package com.ma5951.utils;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class State {

    private Supplier<Boolean> condition;
    private Command command;
    
    private boolean runInit;
    private boolean runEnd;

    public State(Supplier<Boolean> Condition , Command Command ) {
        condition = Condition;
        command = Command;
        runInit = true;
        runEnd = true;
    }

    public void runState() {
       
       if (condition.get() && runInit) {
            command.initialize();
            runInit = false;      
        } else if (condition.get() && runInit == false) {
            command.execute();
            runEnd = false;
        }  else if (!condition.get() && runEnd == false){        
            command.end((false));
            runInit = true;
        } 
    }

}
