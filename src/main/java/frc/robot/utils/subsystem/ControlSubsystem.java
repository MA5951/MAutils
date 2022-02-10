package frc.robot.utils.subsystem;

public interface ControlSubsystem extends MotorSubsystem{
    public void setSetpoint(double setPoint);

    public boolean atSetpoint();

    default double calculate(double setpoint){
        setSetpoint(setpoint);
        return calculate();
    }

    public double calculate();
}