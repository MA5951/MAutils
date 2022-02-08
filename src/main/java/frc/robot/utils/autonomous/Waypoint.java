// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Waypoint{
    public double x, y;
    public double targetVelocity;
    public double curvature, distanceAlongPath;
    public boolean reverse;

    public Waypoint(double x, double y) {
        this.x = x;
        this.y = y;
        this.reverse = false;
    }

    public Waypoint(double x, double y, boolean reverse) {
        this.x = x;
        this.y = y;
        this.reverse = reverse;
    }

    /**
     * returns the distance from the given point to {@code this}.
     * @param other the point to calculate the distance from
     * @return the distance between the points
     */
    public double getDistance(Waypoint other) {
        return Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
    }

    /**
     * returns the dot product between the given point to {@code this}.
     * @param other the point to calculate the dot product with
     * @return the dot product between the points
    */
    public double getDotProduct(Waypoint other) {
        return this.x*other.x + this.y*other.y;
    }

    /**
     * returns the vector addition between the given point to {@code this}.
     * @param other the point to calculate the addition with
     * @return the addition between the points
    */
    Waypoint plus(Waypoint other) {
        return new Waypoint(this.x + other.x, this.y + other.y);
    }

    /**
     * returns the vector subtraction between the given point to {@code this}.
     * @param other the point to calculate the subtraction with
     * @return the subtraction between the points
    */
    public Waypoint minus(Waypoint other) {
        return new Waypoint(this.x - other.x, this.y - other.y);
    }

    /**
     * returns the vector multiplication between the given point to {@code this}.
     * @param other the point to calculate the multiplication with
     * @return the multiplication between the points
    */
    public Waypoint multiply(Waypoint other) {
        return new Waypoint(this.x * other.x, this.y * other.y);
    }

    /**
     * takes a WPILib's {@link Pose2d} and returns it's translation in {@link Waypoint} terms.
     * @param point the target point
     * @return the point in {@link Waypoint} terms
    */
    public static Waypoint FromPose2d(Pose2d point) {
        Translation2d translation = point.getTranslation();
        return new Waypoint(translation.getX(), translation.getY());
    }

    public String toString() {
        return "(" + this.x + ", " + this.y + ")";
    }

	public double[] toArray() {
		return new double[]{x, y};
	}
}
