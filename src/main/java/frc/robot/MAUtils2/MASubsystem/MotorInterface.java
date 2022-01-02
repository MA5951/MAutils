package frc.robot.MAUtils2.MASubsystem;

public interface MotorInterface extends singletonInterface{
    public void setVoltege (double voltege);
    
    default void setPower(double power){
        setVoltege(power * 12);
    }

    default double getVoltege(){
        return getPower() * 12;
    }

    default double getPower();
}