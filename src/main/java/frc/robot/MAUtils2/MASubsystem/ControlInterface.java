package frc.robot.MAUtils2.MASubsystem;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ControlInterface extends MotorInterface {
    public void setSetpoint(double setPoint);

    public boolean atSetpoint();

    default double calculate(double setPoint){
        setSetpoint(setPoint);
        return calculate();
    }

    public double calculate();

    public Subsystem getInstance();
}