package frc.robot.utils.Actuators.MAMotorControllers;

interface MAMotorControlInterface {

    public void setvoltage(double Voltage);

    public void set(double Power);

    public double getOutPut();

    public void configRampRate(double rampRate);

    public void setInverted(Boolean setInverted);

    public void changeMode(boolean onOff);

    public int getID();

    default void setCurrentLimit(int limit) {

    }

    default void resetEncoder() {

    }

    default void phaseSensor(boolean PhaseSensor) {

    }

    default double getPosition() {
        return 0;
    }

    default double getVelocity() {
        return 0;
    }

    default boolean getForwardLimitSwitch() {
        return false;
    }

    default boolean getReverseLimitSwitch() {
        return false;
    }

    default double getStatorCurrent() {
        return 0;
    }

}