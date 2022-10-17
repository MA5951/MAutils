package com.ma5951.utils.swerve;

import java.util.function.Supplier;

public class SwerveConstants {
    private Supplier<Double> wheelRadius;
    private Supplier<Integer> encoderResolution;

    private Supplier<Double> distancePerPulse;
    private Supplier<Double> anglePerPulse;

    private Supplier<Double> moduleMaxAngularVelocity;
    private Supplier<Double> modduleMaxAngularAcceleration;

    public SwerveConstants(Supplier<Double> wheelRadius, Supplier<Integer> encoderResolution, Supplier<Double> distancePerPulse, Supplier<Double> anglePerPulse, Supplier<Double> moduleMaxAngularVelocity, Supplier<Double> modduleMaxAngularAcceleration) {
        this.wheelRadius = wheelRadius;
        this.encoderResolution = encoderResolution;
        this.distancePerPulse = distancePerPulse;
        this.anglePerPulse = anglePerPulse;
        this.moduleMaxAngularVelocity = moduleMaxAngularVelocity;
        this.modduleMaxAngularAcceleration = modduleMaxAngularAcceleration;
    }

    public SwerveConstants(double wheelRadius, int encoderResolution, double distancePerPulse, double anglePerPulse, double moduleMaxAngularVelocity, double modduleMaxAngularAcceleration) {
        this(() -> wheelRadius, () -> encoderResolution, () -> distancePerPulse, () -> anglePerPulse, () -> moduleMaxAngularVelocity, () -> modduleMaxAngularAcceleration);
    }

    public double getWheelRadius() {
        return wheelRadius.get();
    }

    public double getEncoderResolution() {
        return encoderResolution.get();
    }

    public double getDistancePerPulse() {
        return distancePerPulse.get();
    }

    public double getAnglePerPulse() {
        return anglePerPulse.get();
    }

    public double getModuleMaxAngularVelocity() {
        return moduleMaxAngularVelocity.get();
    }

    public double getModduleMaxAngularAcceleration() {
        return modduleMaxAngularAcceleration.get();
    }
}
