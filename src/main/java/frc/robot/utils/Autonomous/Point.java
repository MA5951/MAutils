// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Autonomous;


import frc.robot.subsystems.Chassis.ChassisConstants;

/** Add your docs here. */
public class Point {

    public static enum pointState {
        RIGHT, LEFT, STRAIGHT_LINE, TURN_IN_PLACE
    }

    private Point lasPoint;

    private double distance, angle, maxVelocity = ChassisConstants.KMAX_SPEED,
            acceleration = ChassisConstants.KMAX_ACCELERATION, endVelocity = 0;

    private pointState state = pointState.STRAIGHT_LINE;
    private double timeInMaxSpeed = 0;
    private double accelerationTimeToSetPoint = 0;
    private double accelerationTimeToMaxSpeed = 0;
    private double distancePassInAccelerationMove = 0;
    private double distancePassInAccelerationToSetPoint = 0;
    private double timeInPoint = 0;
    private double rightCircelRadius = 0;
    private double leftCircelRadius = 0;

    public Point(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;

    }

    public Point(double distance, double angle, double maxVelocity, double endVelocity) {
        this.distance = distance;
        this.angle = angle;
        this.endVelocity = endVelocity;
        this.maxVelocity = maxVelocity;
        this.acceleration = acceleration * Math.abs(maxVelocity) / maxVelocity;

    }

    public Point(double distance, double angle, double maxVelocity, double endVelocity, double acceleration) {
        this.distance = distance;
        this.angle = angle;
        this.maxVelocity = maxVelocity;
        this.endVelocity = endVelocity;
        this.acceleration = acceleration;
    }

    public double getDeltaTheta() {
        return Math.toRadians(getAngle() - lasPoint.getAngle());
    }

    public double getEndVelocity() {
        return endVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getTimeInPoint() {
        return timeInPoint;
    }

    public void setTimeInPoint(double timeInPoint) {
        this.timeInPoint = timeInPoint;
    }

    public double getDistancePassInAccelerationToSetPoint() {
        return distancePassInAccelerationToSetPoint;
    }

    public double getDistancePassInAccelerationMove() {
        return distancePassInAccelerationMove;
    }

    public double getAccelerationTimeToMaxSpeed() {
        return accelerationTimeToMaxSpeed;
    }

    public double getAccelerationTimeToSetPoint() {
        return accelerationTimeToSetPoint;
    }

    public double getTimeInMaxSpeed() {
        return timeInMaxSpeed;
    }

    public void setLastPoint(Point point) {
        lasPoint = point;
    }

    public Point getLastPoint() {
        return lasPoint;
    }

    public void calculatTimeAndDistanceToAutonomous() {
        accelerationTimeToSetPoint = Math.abs((getMaxVelocity() - getEndVelocity()) / getAcceleration());

        accelerationTimeToMaxSpeed = Math.abs((getMaxVelocity() - lasPoint.getEndVelocity()) / getAcceleration());

        distancePassInAccelerationMove = lasPoint.getEndVelocity() * accelerationTimeToMaxSpeed
                + (getAcceleration() / 2) * Math.pow(accelerationTimeToMaxSpeed, 2);

        distancePassInAccelerationToSetPoint = getMaxVelocity() * accelerationTimeToSetPoint
                - (getAcceleration() / 2) * Math.pow(accelerationTimeToSetPoint, 2);

        if (state == pointState.RIGHT || state == pointState.LEFT) {
            timeInMaxSpeed = Math
                    .abs((getArcDistance() - (distancePassInAccelerationToSetPoint + distancePassInAccelerationMove))
                            / getMaxVelocity());
        } else {
            timeInMaxSpeed = Math
                    .abs((getDistance() - (distancePassInAccelerationToSetPoint + distancePassInAccelerationMove))
                            / getMaxVelocity());
        }

    }

    public void changeSpeedByAngularVelocity() {
        double anglePassInAcceleratio = Math.toRadians(angle) - ((distancePassInAccelerationMove / getCircelRadius())
                + (distancePassInAccelerationToSetPoint / getCircelRadius()));
        double angularVelocity = anglePassInAcceleratio / timeInMaxSpeed;
        maxVelocity = angularVelocity * ChassisConstants.KwhellRadius;
    }

    public void setCircelRaduis() {
        if (state == pointState.RIGHT) {
            leftCircelRadius = Math.sqrt((Math.pow(getDistance(), 2) / (2 - 2 * Math.cos(getDeltaTheta()))));
            rightCircelRadius = leftCircelRadius - ChassisConstants.KchassisLength;
        } else if (state == pointState.LEFT) {
            rightCircelRadius = Math.sqrt((Math.pow(getDistance(), 2) / (2 - 2 * Math.cos(getDeltaTheta()))));
            leftCircelRadius = rightCircelRadius - ChassisConstants.KchassisLength;
        } else if (state == pointState.TURN_IN_PLACE) {
            leftCircelRadius = Math.sqrt((Math.pow(getDistance(), 2) / (2 - 2 * Math.cos(getDeltaTheta()))));
            rightCircelRadius = leftCircelRadius;
        } else {
            leftCircelRadius = 0;
            rightCircelRadius = leftCircelRadius;
        }
    }

    public double getCircelRadius() {
        if (state == pointState.RIGHT) {
            return leftCircelRadius;
        } else if (state == pointState.LEFT) {
            return rightCircelRadius;
        } else if (state == pointState.TURN_IN_PLACE) {
            return leftCircelRadius;
        }
        return 0;
    }

    public pointState getState() {
        return state;
    }

    public void setState() {
        double sight = getDistance() / Math.abs(getDistance());
        if (getDistance() == lasPoint.getDistance()) {
            state = pointState.TURN_IN_PLACE;
        } else if ((getAngle() - lasPoint.getAngle()) * sight > 0) {
            state = pointState.RIGHT;
        } else if ((getAngle() - lasPoint.getAngle()) * sight < 0) {
            state = pointState.LEFT;
        } else if (getAngle() == lasPoint.getAngle()) {
            state = pointState.STRAIGHT_LINE;
        }
    }

    public double getArcDistance() {
        if (state == pointState.RIGHT) {
            return leftCircelRadius * getDeltaTheta();
        } else if (state == pointState.LEFT) {
            return rightCircelRadius * getDeltaTheta();
        } else if (state == pointState.TURN_IN_PLACE) {
            return leftCircelRadius * getDeltaTheta();
        } else {
            return 0;
        }
    }

}
