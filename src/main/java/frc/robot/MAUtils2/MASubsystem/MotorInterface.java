package frc.robot.MAUtils2.MASubsystem;

public interface MotorInterface extends singletonInterface{
    public void setVoltege (double voltege);
    
    default void setPower(double power){
        setVoltege(power * 12);
    }

    public double getVoltege();

    default double getPower(){
        return getVoltege() / 12;
    }
}