Step 1:
Copy the StateControl subsystem to your robot code subsystem folder and the StateRunner command and NullCommand to your commands folder.

Step 2:
Copy the defult command to run the StateRunner command to your robot init:


CommandScheduler.getInstance().setDefaultCommand(
        StateControl.getInstance(), new StateRunner() );


Step 3:
Create your subsystem states in your desierd place and pass them to the StateRunner command (The last one is in the highst priority)


If none of the states are active the controller will run NullCommand




