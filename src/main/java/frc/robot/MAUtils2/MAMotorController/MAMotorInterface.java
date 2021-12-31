package frc.robot.MAUtils2.MAMotorController;

interface MAMotorControlInterface {

    public void setvoltage(double Voltage);

    public void set(double Power);

    public double getOutPut();

    public void configRampRate(double rampRate);

    public void setInverted(boolean setInverted);

    public void changeMode(boolean onOff);

    public int getID();

    public void follow(MAMotorController toFollow); //TODO considere to write follow in each class because Victor can't follow SparkMax
}