package com.ma5951.utils;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Logger {
    private static Logger logger;
    private final DataLog log;
    private final DoubleArrayLogEntry odometryLogs;
    private final DoubleArrayLogEntry swerveStatesLogs;
    private final HashMap<String, StringLogEntry> messages;

    private Logger() {
        DataLogManager.start();
        log = DataLogManager.getLog();
        odometryLogs = new DoubleArrayLogEntry(log, "odometry");
        swerveStatesLogs = new DoubleArrayLogEntry(log, "swerve states");
        messages = new HashMap<String, StringLogEntry>();
    }

    public void logOdometry(Pose2d pose) {
        double[] odometryLog = new double[3];
        odometryLog[0] = pose.getX();
        odometryLog[1] = pose.getY();
        odometryLog[2] = pose.getRotation().getRadians();
        odometryLogs.append(odometryLog);
    }

    public void logswerveState(SwerveModuleState[] swerveModuleState) {
        double[] swerveModuleStateLog = new double[8];
        for (int i = 0; i < swerveModuleStateLog.length / 2; i++) {
            int j = i * 2;
            swerveModuleStateLog[j] = swerveModuleState[i].angle.getRadians();
            swerveModuleStateLog[j + 1] = swerveModuleState[i].speedMetersPerSecond;
        }
        swerveStatesLogs.append(swerveModuleStateLog);
    }

    public void logMessage(String logEntry, String message) {
        if (messages.containsKey(logEntry)) {
            messages.get(logEntry).append(message);
        } else {
            messages.put(logEntry, 
                new StringLogEntry(log, logEntry));
            messages.get(logEntry).append(message);
        }
    }

    public static Logger getInstance() {
        if (logger == null) {
            logger = new Logger();
        }
        return logger;
    }
    
}
