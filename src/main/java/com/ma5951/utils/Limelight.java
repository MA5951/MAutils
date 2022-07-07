/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.ma5951.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
  private double KDELTA_Y = 0;
  private double KLIMELIGHT_ANGLE = 0;
  private double x;
  private double y;
  private boolean v;
  private double targetArea;
  private double skew ;
  private double l;
  private double Tshort;
  private double Tlong;
  private double Thor;
  private double Tvert;
  private double yaw;
  private double distanceFromTargetLimelightX;
  private double distanceFromTargetLimelightY;
  private double pipe;

  private NetworkTable table;
  private NetworkTableEntry threeDimension = table.getEntry("camtran");
  private NetworkTableEntry tx = table.getEntry("tx");//Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
  private NetworkTableEntry ty = table.getEntry("ty");//Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
  private NetworkTableEntry ta = table.getEntry("ta");//Target Area (0% of image to 100% of image)
  private NetworkTableEntry tv = table.getEntry("tv");//Whether the limelight has any valid targets (0 or 1)
  private NetworkTableEntry ts = table.getEntry("ts");//Skew or rotation (-90 degrees to 0 degrees)
  private NetworkTableEntry tl = table.getEntry("tl");//The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
  private NetworkTableEntry tlong = table.getEntry("tlong");//Sidelength of longest side of the fitted bounding box (pixels)
  private NetworkTableEntry tshort = table.getEntry("tshort");//Sidelength of shortest side of the fitted bounding box (pixels)
  private NetworkTableEntry thor = table.getEntry("thor");//Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  private NetworkTableEntry tvert = table.getEntry("tvert");//Vertical sidelength of the rough bounding box (0 - 320 pixels)
  private NetworkTableEntry getpipe = table.getEntry("getpipe");//True active pipeline index of the camera (0 .. 9)

  public Limelight(String tableName,double KDELTA_Y, double KLIMELIGHT_ANGLE){
     table = NetworkTableInstance.getDefault().getTable(tableName);
     this.KDELTA_Y = KDELTA_Y;
     this.KLIMELIGHT_ANGLE = KLIMELIGHT_ANGLE;
  }

  public double distance() {
    double limelightAngle = y + KLIMELIGHT_ANGLE;
    return Math.tan(limelightAngle) / KDELTA_Y;
  }

  public void setLedMode(int ledMode) {
    table.getEntry("ledMode").setNumber(ledMode);
  }

  public void setCamMode(int camMode) {
    table.getEntry("camMode").setNumber(camMode);
  }

  public void setPipeline(int pipeline) {
    table.getEntry("camMode").setNumber(pipeline);
  }

  public void setStream(int stream) {
    table.getEntry("stream").setNumber(stream);
  }

  public double getX(){
    return this.x;
  }

  public double getY(){
    return this.y;
  }

  public double getA(){
    return this.targetArea;
  }

  public Boolean getV(){
    return this.v;
  }

  public double getS(){
    return this.skew;
  }

  public double getL(){
    return this.l;
  }

  public double getPipe(){
    return this.pipe;
  }

  public double getTlong(){
    return this.Tlong;
  }

  public double getThor(){
    return this.Thor;
  }

  public double getTvert(){
    return this.Tvert;
  }

  public double getTshort(){
    return this.Tshort;
  }

  public double getYaw(){
    return this.yaw;
  }

  public double getDistanceFromTargetLimelightX(){
    return this.distanceFromTargetLimelightX;
  }

  public double getDistanceFromTargetLimelightY(){
    return this.distanceFromTargetLimelightY;
  }

  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    targetArea = ta.getDouble(0.0);
    v = tv.getBoolean(false);
    skew = ts.getDouble(0.0);
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