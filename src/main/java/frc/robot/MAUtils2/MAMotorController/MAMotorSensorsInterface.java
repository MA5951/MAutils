package frc.robot.MAUtils2.MAMotorController;

public interface MAMotorSensorsInterface {
    
    public void resetEncoder();

    public double getPosition();

    public double getVelocity();

    public double getStatorCurrent();
}
