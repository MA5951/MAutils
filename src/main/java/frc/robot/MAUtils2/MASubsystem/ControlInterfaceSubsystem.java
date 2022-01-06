package frc.robot.MAUtils2.MASubsystem;

public interface ControlInterfaceSubsystem extends MotorInterfaceSubsystem{
    public void setSetpoint(double setPoint);

    public boolean atSetpoint();

    default double calculate(double setPoint){
        setSetpoint(setPoint);
        return calculate();
    }

    public double calculate();
}