package frc.robot.utils.MAMotorControlrs;

public interface MAMotorControlInterface {

    public void setvoltage(double Volteg);

    public void set(double Power);

    public double getOutPut();

    public void configRampRate(double rampRate);

    public void setInverted(Boolean setInverted);

    public void changeMood(boolean onOff);

    public int getID();


    default void setCurrentLimit(int limit){

    }

    default void resetEncoder() {

    }

    default void PhaseSensor(boolean PhaseSensor) {

    }

    default double getPosition() {
        return 0;
    }

    default double getVelocity() {
        return 0;
    }

    default boolean getForwardLimitSwitch() {
        return true;
    }

    default boolean getReversLimitSwitch() {
        return true;
    }

    default double getStatorCurrent() {
        return 0;
    }

}