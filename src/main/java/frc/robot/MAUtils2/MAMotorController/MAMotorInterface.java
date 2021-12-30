package frc.robot.MAUtils2.MAMotorController;

interface MAMotorControlInterface {

    public void setvoltage(double Voltage);

    public void set(double Power);

    public double getOutPut();

    public void configRampRate(double rampRate);

    public void setInverted(Boolean setInverted);

    public void changeMode(boolean onOff);

    public int getID();
}