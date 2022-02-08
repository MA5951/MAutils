package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public interface MATalonSRXInterface {
    
    public void configForwardLimitSwitchSource();

    public void configReverseLimitSwitchSource();

    public void resetOnLimitF(boolean limit);

    public void resetOnLimitR(boolean limit);

    public void overrideLimitSwitches(boolean override);

    public void setFeedBack(FeedbackDevice FD);
}
