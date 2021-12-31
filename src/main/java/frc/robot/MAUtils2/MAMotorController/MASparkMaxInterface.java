package frc.robot.MAUtils2.MAMotorController;

import com.revrobotics.CANSparkMax;

public interface MASparkMaxInterface {
    
    public void setCanEncoder();

    public void setCanAlternateEncoder();

    public CANSparkMax getCanSparkMax();

    public void enableLimitSwitchR(boolean enable);

    public void enableLimitSwitchF(boolean enable);
}
