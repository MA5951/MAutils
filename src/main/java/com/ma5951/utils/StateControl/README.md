Step 1:
Copy the StateControl subsystem to your robot code subsystem folder and the StateRunner command to your commands folder.

Step 2:
Copy the defult command to run the StateRunner command to your robot init:

CommandScheduler.getInstance().setDefaultCommand(
        StateControl.getInstance(), new StateRunner() );
  }


Step 3:
Create your subsystem states in the StateRunner command (line 15)


Step 4:
Call the .runState() function for every state you created in the execute section in the StateRunner command (By priorit)

