package frc.robot.utils.subsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.RobotConstants;

public interface MotorInterfaceSubsystem extends Subsystem {
    public void setVoltage (double voltege);
    
    default void setPower(double power){
        setVoltage(power * RobotConstants.voltage);
    }

    public double getVoltage();

    default double getPower(){
        return getVoltage() * RobotConstants.voltage;
    }
}