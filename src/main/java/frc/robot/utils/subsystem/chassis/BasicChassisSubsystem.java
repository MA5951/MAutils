package frc.robot.utils.subsystem.chassis;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface BasicChassisSubsystem extends Subsystem{
    public void setLeftVoltage(double voltage);

    public void setRightVoltage(double voltage);

    default void setLeftPercent(double power){
        setLeftVoltage(power * 12);
    }

    default void setRightPercent(double power){
        setRightVoltage(power *12);
    }

    public void setIdleMode(Boolean Break);
}
