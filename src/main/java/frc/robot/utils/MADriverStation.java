
package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.MAShuffleboard.MAShuffleboard;

/** Add your docs here. */
public class MADriverStation {
    private String Alliance = "";
    private int location = 0;
    private MAShuffleboard driverStationShuffleboard = new MAShuffleboard("SmartDashboard");
    private static MADriverStation maDriverStation;

    private MADriverStation() {
        Alliance = DriverStation.getInstance().getAlliance().toString();
        location = DriverStation.getInstance().getLocation();
    }

    public static void PrintError(String error, boolean printTrace) {
        DriverStation.reportError(error, printTrace);
    }

    public static void PrintWarning(String error, boolean printTrace) {
        DriverStation.reportWarning(error, printTrace);
    }

    public static String getGameSpecificMessage() {
        return DriverStation.getInstance().getGameSpecificMessage();
    }

    public static boolean isOperatorControlEnabled() {
        return DriverStation.getInstance().isOperatorControlEnabled();
    }

    public static boolean isFMSAttached() {
        return DriverStation.getInstance().isFMSAttached();
    }

    public static boolean isDSAttached() {
        return DriverStation.getInstance().isDSAttached();
    }

    public static boolean isAutonomousEnabled() {
        return DriverStation.getInstance().isAutonomousEnabled();
    }

    public static boolean isJoystickConnected(int Port) {
        return DriverStation.getInstance().isJoystickConnected(Port);
    }

    private void PrintValues() {
        driverStationShuffleboard.addString("Alliance", Alliance);
        driverStationShuffleboard.addNum("location", location);
    }

    public void periodic() {
        PrintValues();
    }

    public static MADriverStation getinstance() {
        if (maDriverStation == null) {
            maDriverStation = new MADriverStation();
        }
        return maDriverStation;
    }

}