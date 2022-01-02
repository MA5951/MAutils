package frc.robot.MAUtils2.MAMotorController;

public interface MAMotorControlInterface {

    public void setvoltage(double Voltage);

    default void set(double Power){
        setvoltage(Power * 12);
    }

    public double getOutPut();

    public void configRampRate(double rampRate);

    public void setInverted(Boolean setInverted);

    public void changeMode(boolean onOff);

    public int getID();
    
}