// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autonomous;

import java.util.List;

/** Add your docs here. */
public class PathFollower {
    private List<Waypoint> path;
    private OdometryHandler odometry;
    private RateLimiter rateLimiter;
    private double lookaheadDistance;
    private double robotWidth;

    private int size;
    private int lastIndex;
    private int lastLookaheadIndex;

    private int sign;

    /**
     * 
     * @param path              path to be followed
     * @param odometry          odometry handler that gives the robot's position
     * @param lookaheadDistance how far to lookahead to follow
     * @param maxRate           max change of speed for the rate limiter
     * @param trackWidth        distance between the wheel of the robot side-to-side
     */
    public PathFollower(List<Waypoint> path, OdometryHandler odometry,
            double lookaheadDistance, double maxRate, double robotWidth) {
        this.path = path;
        this.odometry = odometry;
        this.rateLimiter = new RateLimiter(maxRate);
        this.robotWidth = robotWidth;
        this.lookaheadDistance = lookaheadDistance;
        this.size = path.size();
        this.lastIndex = 1;
        this.lastLookaheadIndex = 0;
    }

    /**
     * finds closest point (on the autonomous path) to the robot's position
     * 
     * @return the closest point to the robots's position
     */
    public Waypoint getClosestPoint() {
        Waypoint pos = odometry.getCurrentPosition();
        double minDistance = Double.POSITIVE_INFINITY;
        double distance;
        int minIndex = lastIndex;

        for (int i = lastIndex; i < size; i++) {
            distance = path.get(i).getDistance(pos);
            if (distance < minDistance) {
                minIndex = i;
                minDistance = distance;
            }
        }
        lastIndex = minIndex;
        return path.get(minIndex);
    }

    /**
     * calculates the lookahead point
     * 
     * @return the closest point to the robots's position
     */
    public Waypoint getLookaheadPoint() {
        Waypoint robot = odometry.getCurrentPosition();
        for (int i = lastLookaheadIndex; i < size - 1; i++) {
            Waypoint pointOnPath = path.get(i);
            Waypoint nextPointOnPath = path.get(i + 1);
            Waypoint segment = nextPointOnPath.minus(pointOnPath);
            Waypoint localRobot = pointOnPath.minus(robot);

            double a = segment.getDotProduct(segment);
            double b = 2 * localRobot.getDotProduct(localRobot);
            double c = localRobot.getDotProduct(localRobot) - lookaheadDistance * lookaheadDistance;
            double discriminant = b * b - 4 * a * c;
            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);
                if (t1 >= 0 && t1 <= 1) {
                    lastLookaheadIndex = i;
                    return pointOnPath.plus(segment.multiply(new Waypoint(t1, t1)));
                }
                if (t2 >= 0 && t2 <= 1) {
                    lastLookaheadIndex = i;
                    return pointOnPath.plus(segment.multiply(new Waypoint(t2, t2)));
                }
            }
        }
        return path.get(lastLookaheadIndex);
    }

    private double updateCurvature(Waypoint lookahead) {
        Waypoint robot = odometry.getCurrentPosition();
        if (lookahead == null)
            return Double.POSITIVE_INFINITY;
        double yaw = Math.toRadians(90 - odometry.getYaw());
        double slope = Math.tan(yaw);
        double freeTerm = slope * robot.x - robot.y;
        double x = Math.abs(-slope * lookahead.x + lookahead.y + freeTerm) /
                Math.sqrt(slope * slope + 1); // distance between lookahead point and robot line
        double side = Math.sin(yaw) * (lookahead.x - robot.x) -
                Math.cos(yaw) * (lookahead.y - robot.y); // uses cross product to determine side
        if (side == 0)
            return lookahead.curvature = 0;
        return lookahead.curvature = 2 * x / (lookaheadDistance * lookaheadDistance) * side / Math.abs(side);
    }

    public double[] getSpeeds() {
        Waypoint closestPoint = getClosestPoint();
        Waypoint lookaheadPoint = getLookaheadPoint();
        sign = lookaheadPoint.reverse ? -1 : 1;
        updateCurvature(lookaheadPoint);
        System.out.println("curvature: " + lookaheadPoint.curvature);
        double velocity = closestPoint.targetVelocity;
        velocity = rateLimiter.calculate(closestPoint.targetVelocity);
        return new double[] { (velocity * (2 + lookaheadPoint.curvature * robotWidth) / 2) * sign,
                (velocity * (2 - lookaheadPoint.curvature * robotWidth) / 2) * sign };
    }

    public boolean isDone() {
        return getClosestPoint().equals(path.get(size - 1));
    }
}
