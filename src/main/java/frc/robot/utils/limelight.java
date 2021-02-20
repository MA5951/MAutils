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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelight extends SubsystemBase {
  public static final double KDELTA_Y = 0; // TODO
  private static final double KLIMELIGHT_ANGLE = 0; // TODO
  public static double x;
  public static double y;
  public static boolean v;
  public static double a;
  public static double s;
  public static double l;
  public static double Tshort;
  public static double Tlong;
  public static double Thor;
  public static double Tvert;
  public static double Pipe;
  public static double yaw;
  public static double distanceFromTargetLimelightX;
  public static double distanceFromTargetLimelightY;
  public static double pipe;

  private static NetworkTable table;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry threeDimension;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry ts;
  private static NetworkTableEntry tl;
  private static NetworkTableEntry tlong;
  private static NetworkTableEntry tshort;
  private static NetworkTableEntry thor;
  private static NetworkTableEntry tvert;
  private static NetworkTableEntry getpipe;

  /**
   * Creates a new Limlight.
   */
  private limelight() {

  }

  public static void setLimelightValue() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
    tl = table.getEntry("tl");
    ts = table.getEntry("ts");
    tshort = table.getEntry("tshort");
    tlong = table.getEntry("tlong");
    thor = table.getEntry("thor");
    tvert = table.getEntry("tvert");
    getpipe = table.getEntry("getpipe");
    threeDimension = table.getEntry("camtran");
  }

  public static double distance() {
    double a = y + KLIMELIGHT_ANGLE;
    return Math.tan(a) / KDELTA_Y;
  }

  public static void ledMode(int ledMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledMode);
  }

  public static void camMode(int camMode) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(camMode);
  }

  public static void pipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(pipeline);
  }

  public static void stream(int stream) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(stream);
  }

  @Override
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
