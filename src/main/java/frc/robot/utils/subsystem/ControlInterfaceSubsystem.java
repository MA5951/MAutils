package frc.robot.utils.subsystem;

public interface ControlInterfaceSubsystem extends MotorInterfaceSubsystem{
    public void setSetpoint(double setPoint);

    public boolean atSetpoint();

    default double calculate(double setPoint){
        setSetpoint(setPoint);
        return calculate();
    }

    public double calculate();
}