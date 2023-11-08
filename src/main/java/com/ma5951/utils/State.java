package com.ma5951.utils;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class State {

    private Supplier<Boolean> condition;
    private Command command;
    
    private int commandState;

    public State(Supplier<Boolean> Condition , Command Command ) {
        condition = Condition;
        command = Command;
        commandState = 0;
        
    }

    public void runState() {
       
       if (condition.get() && commandState == 0) {
            command.initialize();
            commandState++;       
        } else if (condition.get() && commandState == 1) {
            command.execute();
            
        } else if ( condition.get() && commandState == 2)
            command.end((false));
            commandState = 0;
        }


}
