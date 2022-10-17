package com.ma5951.utils.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final SwerveConstants swerveConstants;
    private final PIDController drivePID;
    private final ProfiledPIDController turningPID;

    private SimpleMotorFeedforward driveFeedforward;
    private SimpleMotorFeedforward turningFeedforward;

    public SwerveModule(int driveID, int turningID, PIDController drivePID, ProfiledPIDController turningPID, SwerveConstants swerveConstants) {
        this.driveMotor = new TalonFX(driveID);
        this.turningMotor = new TalonFX(turningID);
        this.drivePID = drivePID;
        this.turningPID = turningPID;
        this.swerveConstants = swerveConstants;

        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getSpeed() {
        return driveMotor.getSelectedSensorVelocity() * swerveConstants.getDistancePerPulse();
    }

    public double getAngle() {
        return turningMotor.getSelectedSensorPosition() * swerveConstants.getAnglePerPulse();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), new Rotation2d(getAngle()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

        double driveOutput = drivePID.calculate(getSpeed(), optimizedState.speedMetersPerSecond);
        double driveFeed = driveFeedforward.calculate(optimizedState.speedMetersPerSecond);

        double turnOutput = turningPID.calculate(getAngle(), optimizedState.angle.getRadians());
        double turnFeed = turningFeedforward.calculate(turningPID.getSetpoint().velocity);

        driveMotor.set(ControlMode.Current, driveOutput + driveFeed);
        turningMotor.set(ControlMode.Current, turnOutput + turnFeed);
    }
}