package com.ma5951.utils;

/**
 * Add your docs here.
 */
public class DriverStation {
    private String alliance = "";
    private int location = 0;
    private MAShuffleboard driverStationShuffleboard = new MAShuffleboard("SmartDashboard");
    private static DriverStation driverStation;

    private DriverStation() {
        alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance().toString();
        location = edu.wpi.first.wpilibj.DriverStation.getLocation();
    }

    public static void printError(String error, boolean printTrace) {
        edu.wpi.first.wpilibj.DriverStation.reportError(error, printTrace);
    }

    public static void printWarning(String error, boolean printTrace) {
        edu.wpi.first.wpilibj.DriverStation.reportWarning(error, printTrace);
    }

    public static String getGameSpecificMessage() {
        return edu.wpi.first.wpilibj.DriverStation.getGameSpecificMessage();
    }

    public static boolean isOperatorControlEnabled() {
        return edu.wpi.first.wpilibj.DriverStation.isTeleopEnabled();
    }

    public static boolean isFMSAttached() {
        return edu.wpi.first.wpilibj.DriverStation.isFMSAttached();
    }

    public static boolean isDSAttached() {
        return edu.wpi.first.wpilibj.DriverStation.isDSAttached();
    }

    public static boolean isAutonomousEnabled() {
        return edu.wpi.first.wpilibj.DriverStation.isAutonomousEnabled();
    }

    public static boolean isJoystickConnected(int Port) {
        return edu.wpi.first.wpilibj.DriverStation.isJoystickConnected(Port);
    }

    private void printValues() {
        driverStationShuffleboard.addString("Alliance", alliance);
        driverStationShuffleboard.addNum("location", location);
    }

    public void periodic() {
        printValues();
    }

    public static DriverStation getInstance() {
        if (driverStation == null) {
            driverStation = new DriverStation();
        }
        return driverStation;
    }

}