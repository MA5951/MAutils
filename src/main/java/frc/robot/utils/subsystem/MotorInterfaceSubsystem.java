package frc.robot.utils.subsystem;

public interface MotorInterfaceSubsystem extends SensorInterfaceSubsystem {
    public void setVoltage (double voltege);
    
    default void setPower(double power){
        setVoltage(power * 12);
    }

    public double getVoltage();

    default double getPower(){
        return getVoltage() * 12;
    }
}