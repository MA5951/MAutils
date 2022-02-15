package frc.robot.utils.motor;

public interface MotorController {

    public void setVoltage(double voltage);

    default void setPower(double power){
        setVoltage(power * 12);
    }

    public double getOutput();

    public void configRampRate(double rampRate);

    public void setInverted(Boolean setInverted);

    public void changeMode(boolean onOff);

    public int getID();
    
}