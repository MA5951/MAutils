/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.autonomous;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public class Path implements Serializable {
    public List<Waypoint> points;
    public double spacing;
    public double k;
    public double maxVelocity;
    public double maxAcceleration;
    public double lookaheadDistance;
    public double maxRate;

    public Path(List<Waypoint> points, double spacing, double k, double maxVelocity,
                double maxAcceleration, double lookaheadDistance, double maxRate) {
        this.points = points;
        this.spacing = spacing;
        this.k = k;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.lookaheadDistance = lookaheadDistance;
        this.maxRate = maxRate;
    }

    public void writeToFile(String filePath) {
        try {
            FileOutputStream fos = new FileOutputStream(filePath);
            ObjectOutputStream file = new ObjectOutputStream(fos);
            file.writeObject(this);
            file.close();
          } 
        catch (Exception e){
            System.err.println("Error: " + e.getMessage());
          }
    }

    public static Path readFromFile(String filePath) {
        try {
            FileInputStream fos = new FileInputStream(filePath);
            ObjectInputStream file = new ObjectInputStream(fos);
            Path output = (Path) file.readObject();
            file.close();
            return output;
          } 
        catch (Exception e){
            System.err.println("Error: " + e.getMessage());
            return new Path(null, 0, 0, 0, 0, 0, 0);
          }
    }

    public static class PathBuilder {
        Path path;

        public PathBuilder() {
            path = new Path(new ArrayList<Waypoint>(), 0, 0, 0, 0, 0, 0);
        }

        public Path build() {

            try {
                Path output = (Path) path.clone();
                path = new Path(new ArrayList<Waypoint>(), 0, 0, 0, 0, 0, 0);

                return output;
            }
            catch(Exception e) {
                Path output = new Path(new ArrayList<Waypoint>(), 0, 0, 0, 0, 0, 0);
                path = new Path(new ArrayList<Waypoint>(), 0, 0, 0, 0, 0, 0);
                return output;

            }
        }

        public PathBuilder setWaypoints(List<Waypoint> points) {
            path.points = points;
            return this;
        }
        
        public PathBuilder addPoint(double x, double y) {
            path.points.add(new Waypoint(x, y));
            return this;
        }

        public PathBuilder addPoint(double x, double y, boolean reverse) {
            path.points.add(new Waypoint(x, y, reverse));
            return this;
        }

        public PathBuilder setSpacing(double spacing) {
            path.spacing = spacing;
            return this;
        }

        public PathBuilder setK(double k) {
            path.k = k;
            return this;
        }

        public PathBuilder setMaxVelocity(double maxVelocity) {
            path.maxVelocity = maxVelocity;
            return this;
        }

        public PathBuilder setMaxAcceleration(double maxAcceleration) {
            path.maxAcceleration = maxAcceleration;
            return this;
        }

        public PathBuilder setLookaheadDistance(double lookaheadDistance) {
            path.lookaheadDistance = lookaheadDistance;
            return this;
        }

        public PathBuilder setMaxRate(double maxRate) {
            path.maxRate = maxRate;
            return this;
        }
    }
}