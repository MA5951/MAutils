package frc.robot.utils.motor;

public interface MAMotorSensorsInterface {
    
    public void resetEncoder();

    public double getPosition();

    public double getVelocity();

    public double getStatorCurrent();

    public void phaseSensor(boolean phaseSensor);

    public void setCurrentLimit(int limit);

    public boolean getForwardLimitSwitch();

    public boolean getReverseLimitSwitch();
}
