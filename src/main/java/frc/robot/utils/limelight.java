/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

/**
 * @author yuval rader
 */

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelight {
  public final double KDELTA_Y = 0; // TODO
  private final double KLIMELIGHT_STATIC_ANGLE = 0; // TODO
  public double x;
  public double y;
  public boolean v;
  public double a;
  public double s;
  public double l;
  public double Tshort;
  public double Tlong;
  public double Thor;
  public double Tvert;
  public double Pipe;
  public double yaw;
  public double distanceFromTargetLimelightX;
  public double distanceFromTargetLimelightY;
  public double pipe;

  private NetworkTable table;
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry threeDimension = table.getEntry("camtran");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry ts = table.getEntry("ts");
  private NetworkTableEntry tl = table.getEntry("tl");
  private NetworkTableEntry tlong = table.getEntry("tlong");
  private NetworkTableEntry tshort = table.getEntry("tshort");
  private NetworkTableEntry thor = table.getEntry("thor");
  private NetworkTableEntry tvert = table.getEntry("tvert");
  private NetworkTableEntry getpipe = table.getEntry("getpipe");

  public limelight(String tableName){
     table = NetworkTableInstance.getDefault().getTable(tableName);
  }

  public double distance() {
    double limelightAngle = y + KLIMELIGHT_STATIC_ANGLE;
    return Math.tan(limelightAngle) / KDELTA_Y;
  }

  public void ledMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  public void camMode(int camMode) {
    table.getEntry("camMode").setNumber(camMode);
  }

  public void pipeline(int pipeline) {
    table.getEntry("camMode").setNumber(pipeline);
  }

  public void stream(int stream) {
    table.getEntry("stream").setNumber(stream);
  }

  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);
    v = tv.getBoolean(false);
    s = ts.getDouble(0.0);
    l = tl.getDouble(0.0);
    pipe = getpipe.getDouble(0.0);
    Tlong = tlong.getDouble(0.0);
    Thor = thor.getDouble(0.0);
    Tvert = tvert.getDouble(0.0);
    Tshort = tshort.getDouble(0.0);
    yaw = threeDimension.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0, 0 })[4];
    distanceFromTargetLimelightX = threeDimension.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })[0];
    distanceFromTargetLimelightY = threeDimension.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 })[2];
  }

}