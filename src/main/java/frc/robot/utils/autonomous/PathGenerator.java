package frc.robot.utils.autonomous;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PathGenerator {

    private double spacing;
    private double maxVelocity;
    private double maxAcceleration;
    private double k;
    private double smoothWeight, tolerance;

    private static double really_small = 1 ^ -1000;

    public PathGenerator(double spacing, double k, double maxVelocity, double maxAcceleration) {
        this.spacing = spacing;
        this.k = k;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.smoothWeight = 0.75;
        this.tolerance = 0.006;
    }

    public List<Waypoint> calculate(List<Waypoint> path) {
        ArrayList<Waypoint> newPath = new ArrayList<Waypoint>();
        for (int i = 0; i < path.size() - 1; i++) {
            List<Waypoint> injectedPoints = injectPoints(path.get(i), path.get(i + 1), spacing);
            newPath.addAll(injectedPoints);
        }
        newPath.add(path.get(path.size() - 1));
        System.out.println("inject");
        smooth(newPath);
        System.out.println("smooth");
        updateDistances(newPath);
        System.out.println("distance");
        updateCurvatures(newPath);
        System.out.println("curvature");
        updateMaximumVelocities(newPath);
        System.out.println("max velocities");
        updateVelocities(newPath);
        System.out.println("velocities");
        return newPath;
    }

    private List<Waypoint> injectPoints(Waypoint startPoint, Waypoint endPoint, double spacing) {
        double dxTotal = endPoint.x - startPoint.x;
        double dyTotal = endPoint.y - startPoint.y;
        double distance = Math.sqrt(Math.pow(dxTotal, 2) + Math.pow(dyTotal, 2));
        double newPointsCount = Math.ceil(distance / spacing);
        double dx = dxTotal / newPointsCount;
        double dy = dyTotal / newPointsCount;

        List<Waypoint> injectedPoints = new ArrayList<>();
        for (int i = 0; i < newPointsCount; i++) {
            injectedPoints.add(new Waypoint(startPoint.x + dx * i,
                    startPoint.y + dy * i));
        }
        return injectedPoints;
    }

    private void smooth(List<Waypoint> path) {
        double dataWeight = 1 - smoothWeight;
        double[][] newPath = new double[path.size()][2];
        for (int i = 0; i < path.size(); i++) {
            newPath[i] = path.get(i).toArray();
        }
        double[][] ogPath = new double[path.size()][2];
        for (int i = 0; i < ogPath.length; i++) {
            ogPath[i] = Arrays.copyOf(newPath[i], newPath[i].length);
        }
        double change = tolerance;
        while (change >= tolerance) {
            change = 0;
            System.out.println("3");
            for (int i = 1; i < newPath.length - 1; i++) {
                System.out.println("4 " + i + " " + (newPath.length - 1));
                for (int j = 0; j < newPath[i].length; j++) {
                    System.out.println(i + " " + change);
                    double aux = newPath[i][j];
                    newPath[i][j] += dataWeight * (ogPath[i][j] - newPath[i][j])
                            + smoothWeight * (newPath[i - 1][j] + newPath[i + 1][j] - 2 * newPath[i][j]);
                    change += Math.abs(aux - newPath[i][j]);
                }
            }
        }

        for (int i = 0; i < newPath.length; i++) {
            path.set(i, new Waypoint(newPath[i][0], newPath[i][1]));
        }
    }

    private void updateDistances(List<Waypoint> path) {
        double totalDistance = 0;
        path.get(0).distanceAlongPath = totalDistance;
        for (int i = 1; i < path.size(); i++) {
            Waypoint currentPoint = path.get(i);
            Waypoint lastPoint = path.get(i - 1);
            totalDistance += currentPoint.getDistance(lastPoint);
            currentPoint.distanceAlongPath = totalDistance;
        }
    }

    private void updateCurvatures(List<Waypoint> path) {
        path.get(0).curvature = 0;
        for (int i = 1; i < path.size() - 1; i++) {
            Waypoint currentPoint = path.get(i);
            Waypoint lastPoint = path.get(i - 1);
            Waypoint followingPoint = path.get(i + 1);

            double x1 = currentPoint.x;
            double y1 = currentPoint.y;
            double x2 = lastPoint.x;
            double y2 = lastPoint.y;
            double x3 = followingPoint.x;
            double y3 = followingPoint.y;

            if (x1 == x2) {
                x1 += really_small;
            }

            double k1 = 0.5 * (Math.pow(x1, 2) + Math.pow(y1, 2) - Math.pow(x2, 2) - Math.pow(y2, 2)) / (x1 - x2);
            double k2 = (y1 - y2) / (x1 - x2);

            double b = 0.5 * (Math.pow(x2, 2) - (2 * x2 * k1) + Math.pow(y2, 2) - Math.pow(x3, 2) + (2 * x3 * k1))
                    - Math.pow(y3, 2) /
                            ((x3 * k2) - y3 + y2 - (x2 * k2));
            double a = k1 - (k2 * b);

            double r = Math.sqrt(Math.pow(x1 - a, 2) + Math.pow(y1 - b, 2));
            double curvature = 1 / r;

            if (curvature == Double.NaN) {
                curvature = 0;
            }
            currentPoint.curvature = curvature;
        }
        path.get(path.size() - 1).curvature = 0;
    }

    public void updateMaximumVelocities(List<Waypoint> path) {
        for (Waypoint point : path) {
            point.targetVelocity = Math.min(maxVelocity, k / point.curvature);
            System.out.println("max velocity " + point.targetVelocity);
        }
    }

    public void updateVelocities(List<Waypoint> path) {
        path.get(path.size() - 1).targetVelocity = 0;
        for (int i = path.size() - 2; i >= 0; i--) {
            Waypoint currentPoint = path.get(i);
            Waypoint lastPoint = path.get(i + 1);

            double distance = currentPoint.getDistance(lastPoint);
            currentPoint.targetVelocity = Math.min(currentPoint.targetVelocity,
                    Math.sqrt(Math.pow(lastPoint.targetVelocity, 2) + 2 * maxAcceleration * distance));
        }
    }
}