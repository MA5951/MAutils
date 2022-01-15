
package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

/** Add your docs here. */
public class MADriverStation {
    private String alliance = "";
    private int location = 0;
    private MAShuffleboard driverStationShuffleboard = new MAShuffleboard("SmartDashboard");
    private static DriverStation driverStation;
    private static MADriverStation m_DriverStation;

    private MADriverStation() {
        alliance = driverStation.getAlliance().toString();
        location = driverStation.getLocation();
    }

    public static void printError(String error, boolean printTrace) {
        DriverStation.reportError(error, printTrace);
    }

    public static void printWarning(String error, boolean printTrace) {
        DriverStation.reportWarning(error, printTrace);
    }

    public static String getGameSpecificMessage() {
        return driverStation.getGameSpecificMessage();
    }

    public static boolean isOperatorControlEnabled() {
        return driverStation.isEnabled();
    }

    public static boolean isFMSAttached() {
        return driverStation.isFMSAttached();
    }

    public static boolean isDSAttached() {
        return driverStation.isDSAttached();
    }

    public static boolean isAutonomousEnabled() {
        return driverStation.isAutonomousEnabled();
    }

    public static boolean isJoystickConnected(int Port) {
        return driverStation.isJoystickConnected(Port);
    }

    private void printValues() {
        driverStationShuffleboard.addString("Alliance", alliance);
        driverStationShuffleboard.addNum("location", location);
    }

    public void periodic() {
        printValues();
    }

    public static MADriverStation getinstance() {
        if (m_DriverStation == null) {
            m_DriverStation = new MADriverStation();
        }
        return m_DriverStation;
    }

}