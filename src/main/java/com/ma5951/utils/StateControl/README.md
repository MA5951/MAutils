Step 1:
Copy the StateControl subsystem to your robot code subsystem folder and the StateRunner command to your commands folder.

Step 2:
Copy the defult command to run the StateRunner command to your robot init:

CommandScheduler.getInstance().setDefaultCommand(
        StateControl.getInstance(), new StateRunner() );



Step 3:
Create your subsystem states in your desierd place

Step 4:


