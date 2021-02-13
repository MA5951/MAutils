
package frc.robot.utils.MAShuffleboard;

import frc.robot.utils.MAPidController;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

/**
 * @author yuval rader
 */

public class MAShuffleboard {
  private ShuffleboardNetworkTableEntry NumValue;
  private ShuffleboardNetworkTableEntry StringValue;
  private ShuffleboardNetworkTableEntry BooleanValue;
  private ShuffleboardNetworkTableEntry PID;

  private Map<String, ShuffleboardNetworkTableEntry> NetworkTableMap = new HashMap<String, ShuffleboardNetworkTableEntry>();
  private Map<String, Boolean> booleanMap = new HashMap<String, Boolean>();
  private static String currnetTab;

  public MAShuffleboard(String tab) {
    currnetTab = tab;
  }

  public void addNum(String title, double value) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }

    if (booleanMap.get(title)) {
      NumValue = new ShuffleboardNetworkTableEntry(title, 0, currnetTab);
      NetworkTableMap.put(title, NumValue);
      booleanMap.put(title, false);
    } else {
      NetworkTableMap.get(title).setValue(value);
    }

  }

  public void addString(String title, String value) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      StringValue = new ShuffleboardNetworkTableEntry(title, "", currnetTab);
      NetworkTableMap.put(title, StringValue);
      booleanMap.put(title, false);
    } else {
      NetworkTableMap.get(title).setValue(value);
    }
  }

  public void addBoolean(String title, Boolean value) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      BooleanValue = new ShuffleboardNetworkTableEntry(title, false, currnetTab);
      NetworkTableMap.put(title, BooleanValue);
      booleanMap.put(title, false);
    } else {
      NetworkTableMap.get(title).setValue(value);
    }
  }

  public void addNum(String title, double value, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }

    if (booleanMap.get(title)) {
      NumValue = new ShuffleboardNetworkTableEntry(title, 0, currnetTab, widgetType);
      NetworkTableMap.put(title, NumValue);
      booleanMap.put(title, false);
    } else {
      NetworkTableMap.get(title).setValue(value);
    }

  }

  public void addString(String title, String value, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      StringValue = new ShuffleboardNetworkTableEntry(title, "", currnetTab, widgetType);
      NetworkTableMap.put(title, StringValue);
      booleanMap.put(title, false);
    } else {
      NetworkTableMap.get(title).setValue(value);
    }
  }

  public void addBoolean(String title, Boolean value, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      BooleanValue = new ShuffleboardNetworkTableEntry(title, false, currnetTab, widgetType);
      NetworkTableMap.put(title, BooleanValue);
      booleanMap.put(title, false);
    } else {
      NetworkTableMap.get(title).setValue(value);
    }
  }

  public boolean getBolean(String title) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      BooleanValue = new ShuffleboardNetworkTableEntry(title, false, currnetTab);
      NetworkTableMap.put(title, BooleanValue);
      booleanMap.put(title, false);
    }
    return BooleanValue.getBooleanValue(false);
  }

  public String getString(String title) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      StringValue = new ShuffleboardNetworkTableEntry(title, false, currnetTab);
      NetworkTableMap.put(title, StringValue);
      booleanMap.put(title, false);
    }
    return StringValue.getStringValue("");
  }

  public double getNum(String title) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      NumValue = new ShuffleboardNetworkTableEntry(title, false, currnetTab);
      NetworkTableMap.put(title, NumValue);
      booleanMap.put(title, false);
    }
    return NumValue.getNumValue(0.0);
  }

  public boolean getBolean(String title, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      BooleanValue = new ShuffleboardNetworkTableEntry(title, false, currnetTab, widgetType);
      NetworkTableMap.put(title, BooleanValue);
      booleanMap.put(title, false);
    }
    return BooleanValue.getBooleanValue(false);
  }

  public String getString(String title, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      StringValue = new ShuffleboardNetworkTableEntry(title, "", currnetTab, widgetType);
      NetworkTableMap.put(title, StringValue);
      booleanMap.put(title, false);
    }
    return StringValue.getStringValue("");
  }

  public double getNum(String title, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      NumValue = new ShuffleboardNetworkTableEntry(title, 0.0, currnetTab, widgetType);
      NetworkTableMap.put(title, NumValue);
      booleanMap.put(title, false);
    }
    return NumValue.getNumValue(0.0);
  }

  public void PID(String title, MAPidController pid) {
    if (NetworkTableMap.get(title) == null) {
      booleanMap.put(title, true);
    }
    if (booleanMap.get(title)) {
      PID = new ShuffleboardNetworkTableEntry(title, currnetTab);
      NetworkTableMap.put(title, PID);
      booleanMap.put(title, false);
    }
    PID.SetPIDValuse(pid);
  }

}