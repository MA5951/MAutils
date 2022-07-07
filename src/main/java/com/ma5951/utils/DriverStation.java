package com.ma5951.utils;

/**
 * Adding utilities for the driver station
 */
public class DriverStation {
    private String alliance = "";
    private int location = 0;
    private Shuffleboard driverStationShuffleboard = new Shuffleboard("SmartDashboard");
    private static DriverStation driverStation;

    /**
     * Creating a new Driver Station object
     */

    private DriverStation() {
        alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance().toString();
        location = edu.wpi.first.wpilibj.DriverStation.getLocation();
    }

    /**
     * Printing error to the Driver Station
     * @param error Error to print
     * @param printTrace To print the stack trace or not
     */

    public static void printError(String error, boolean printTrace) {
        edu.wpi.first.wpilibj.DriverStation.reportError(error, printTrace);
    }

    /**
     * Printing warning to the Driver Station
     * @param error Error to print
     * @param printTrace To print the stack trace or not
     */

    public static void printWarning(String error, boolean printTrace) {
        edu.wpi.first.wpilibj.DriverStation.reportWarning(error, printTrace);
    }

    /**
     * Getting the game spacific message from the FMS
     * @return The game spacific message
     */

    public static String getGameSpecificMessage() {
        return edu.wpi.first.wpilibj.DriverStation.getGameSpecificMessage();
    }

    /**
     * Check if Telleop is enabled
     * @return if Telleop is enabled
     */

    public static boolean isOperatorControlEnabled() {
        return edu.wpi.first.wpilibj.DriverStation.isTeleopEnabled();
    }

    /**
     * Check if the FMS is attached
     * @return if the FMS is attached
     */

    public static boolean isFMSAttached() {
        return edu.wpi.first.wpilibj.DriverStation.isFMSAttached();
    }

    /**
     * Check if the Driver Station is attached
     * @return if the Driver Station is attached
     */

    public static boolean isDSAttached() {
        return edu.wpi.first.wpilibj.DriverStation.isDSAttached();
    }
    
    /**
     * Getting if Autonomous is enabled
     * @return if Autonomous is enabled
     */

    public static boolean isAutonomousEnabled() {
        return edu.wpi.first.wpilibj.DriverStation.isAutonomousEnabled();
    }

    /**
     * Check if there is a joystick connected to specific port
     * @param Port The port to check
     * @return if there is a joystick that connected to the port
     */

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

    /**
     * Singletone
     * @return the Driver Station instance
     */

    public static DriverStation getInstance() {
        if (driverStation == null) {
            driverStation = new DriverStation();
        }
        return driverStation;
    }

}