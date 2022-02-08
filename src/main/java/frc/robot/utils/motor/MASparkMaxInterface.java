package frc.robot.utils.motor;

import com.revrobotics.CANSparkMax;

public interface MASparkMaxInterface {
    
    public void setCanEncoder();

    public void setCanAlternateEncoder();

    public CANSparkMax getCanSparkMax();

    public void enableLimitSwitchR(boolean enable);

    public void enableLimitSwitchF(boolean enable);
}
