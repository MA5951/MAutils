package com.ma5951.utils.StateControl;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

public class State {

    private Supplier<Boolean> condition;
    private Command command;
    private boolean runInit;
    private boolean runEnd;
    private boolean conditionState;

    public State(Supplier<Boolean> Condition , Command Command) {
        condition = Condition;
        command = Command;
        runInit = true;
        runEnd = false;
    }

    public void runState() {

       if (condition.get() && runInit) {
            command.initialize();
            runInit = false;      
        } else if (condition.get() || !command.isFinished() && runInit == false) {
            command.execute();
            runEnd = true;
        }  else if (!condition.get() || command.isFinished() && runEnd ){
            command.end((false));
            runInit = true;
            runEnd = false;

        } 
    }

    public Boolean getConditionState() {
        return condition.get();
    }
    

}
