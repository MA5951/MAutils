package frc.robot.MAUtils2.MASubsystem;

public interface ControlInterface extends singletonInterface{
    public void setSetpoint(double setPoint);

    public boolean atSetpoint();

    default double calculate(double setPoint){
        setSetpoint(setPoint);
        return calculate();
    }

    public double calculate();
}