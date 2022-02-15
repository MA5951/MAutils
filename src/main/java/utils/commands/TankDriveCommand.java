package frc.robot.utils.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.subsystem.chassis.BaseChassisPIDSubsystem;

public class TankDriveCommand extends CommandBase {
    private BaseChassisPIDSubsystem chassis;
    private Joystick leftJoystick;
    private Joystick rightJoystick;

    public static final double K_THRESHOLD = 0.1;
    public static final double K_SCALE = 0.3;

    public TankDriveCommand(BaseChassisPIDSubsystem chassis, Joystick leftJoystick, Joystick rightJoystick) {
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        this.chassis = chassis;
        addRequirements(chassis);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (leftJoystick.getY() > K_THRESHOLD
                || leftJoystick.getY() < -K_THRESHOLD) {

            if (rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1)) {

                chassis.setLeftPercent(leftJoystick.getY() * K_SCALE);
            } else {
                chassis.setLeftPercent(leftJoystick.getY());
            }
        } else {

            chassis.setLeftPercent(0);
        }


        if (rightJoystick.getY() > K_THRESHOLD
                || rightJoystick.getY() < -K_THRESHOLD) {
            if (rightJoystick.getRawButton(1) || leftJoystick.getRawButton(1)) {

                chassis.setRightPercent(rightJoystick.getY() * K_SCALE);
            } else {
                chassis.setRightPercent(rightJoystick.getY());
            }
        } else {

            chassis.setRightPercent(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        chassis.setLeftPercent(0);
        chassis.setRightPercent(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
