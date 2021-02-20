// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.MAShuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.utils.MAPidController;

/** Add your docs here. */
class ShuffleboardNetworkTableEntry {
    private NetworkTableEntry NumValue;
    private NetworkTableEntry StringValue;
    private NetworkTableEntry BooleanValue;
    private NetworkTableEntry KP;
    private NetworkTableEntry KI;
    private NetworkTableEntry KD;
    private NetworkTableEntry setPoint;

    public ShuffleboardNetworkTableEntry(String title, String currnetTab) {
        ShuffleboardLayout PID = Shuffleboard.getTab(currnetTab).getLayout(title, BuiltInLayouts.kList);
        KP = PID.add("KP", 0).getEntry();
        KI = PID.add("KI", 0).getEntry();
        KD = PID.add("KD", 0).getEntry();
        setPoint = PID.add("SetPoint", 0).getEntry();
    }

    public ShuffleboardNetworkTableEntry(String title, Object DV, String currnetTab) {
        if (DV instanceof Boolean) {
            BooleanValue = Shuffleboard.getTab(currnetTab).add(title, DV).getEntry();
        } else if (DV instanceof Number) {
            NumValue = Shuffleboard.getTab(currnetTab).add(title, DV).getEntry();
        } else if (DV instanceof String) {
            StringValue = Shuffleboard.getTab(currnetTab).add(title, DV).getEntry();
        }

    }

    public ShuffleboardNetworkTableEntry(String title, Object DV, String currnetTab, WidgetType widgetType) {
        if (DV instanceof Boolean) {
            BooleanValue = Shuffleboard.getTab(currnetTab).add(title, DV).withWidget(widgetType).getEntry();
        } else if (DV instanceof Number) {
            NumValue = Shuffleboard.getTab(currnetTab).add(title, DV).withWidget(widgetType).getEntry();
        } else if (DV instanceof String) {
            StringValue = Shuffleboard.getTab(currnetTab).add(title, DV).withWidget(widgetType).getEntry();
        }

    }

    public void setValue(Number value) {
        NumValue.setNumber(value);
    }

    public void setValue(String value) {
        StringValue.setString(value);
    }

    public void setValue(Boolean value) {
        BooleanValue.setBoolean(value);
    }

    public double getNumValue(double DV) {
        return NumValue.getDouble(DV);
    }

    public String getStringValue(String DV) {
        return StringValue.getString(DV);
    }

    public boolean getBooleanValue(boolean DV) {
        return BooleanValue.getBoolean(DV);
    }

    public void SetPIDValuse(MAPidController pid) {
        pid.setPID(KP.getDouble(0.0), KI.getDouble(0.0), KD.getDouble(0.0));
        pid.setSetpoint(setPoint.getDouble(0.0));
    }

}
