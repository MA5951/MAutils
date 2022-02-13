package frc.robot.utils.subsystem.chassis;

public interface BasicChassisPIDSubsystem extends BasicChassisSubsystem {
    public boolean isPIDRightVelocityAtSetPoint();

    public boolean isPIDLeftVelocityAtSetPoint();

    public void setRightVelocitySetpoint(double setPoint);

    public void setLeftVelocitySetpoint(double setPoint);

    public double getRightPID();

    public double getLeftPID();

}
