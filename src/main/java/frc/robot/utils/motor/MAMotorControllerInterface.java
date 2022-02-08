package frc.robot.utils.motor;

public interface MAMotorControllerInterface {

    public void setvoltage(double Voltage);

    default void set(double Power){
        setvoltage(Power * 12);
    }

    public double getOutput();

    public void configRampRate(double rampRate);

    public void setInverted(Boolean setInverted);

    public void changeMode(boolean onOff);

    public int getID();
    
}