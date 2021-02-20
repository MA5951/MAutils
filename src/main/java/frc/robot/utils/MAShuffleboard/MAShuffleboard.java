
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
  private static String currnetTab;

  public MAShuffleboard(String tab) {
    currnetTab = tab;
  }

  public ShuffleboardNetworkTableEntry MAShugglebordPattern(ShuffleboardNetworkTableEntry valEntry, String title,
      Object DV) {
    if (NetworkTableMap.get(title) == null) {
      valEntry = new ShuffleboardNetworkTableEntry(title, DV, currnetTab);
      NetworkTableMap.put(title, valEntry);
    }
    return NetworkTableMap.get(title);
  }

  public ShuffleboardNetworkTableEntry MAShugglebordPattern(ShuffleboardNetworkTableEntry valEntry, String title,
      Object DV, WidgetType widgetType) {
    if (NetworkTableMap.get(title) == null) {
      valEntry = new ShuffleboardNetworkTableEntry(title, DV, currnetTab, widgetType);
      NetworkTableMap.put(title, valEntry);
    }
    return NetworkTableMap.get(title);
  }

  public void addNum(String title, double value) {
    MAShugglebordPattern(NumValue, title, 0).setValue(value);
  }

  public void addString(String title, String value) {
    MAShugglebordPattern(StringValue, title, "").setValue(value);
  }

  public void addBoolean(String title, Boolean value) {
    MAShugglebordPattern(BooleanValue, title, false).setValue(value);
  }

  public void addNum(String title, double value, WidgetType widgetType) {
    MAShugglebordPattern(NumValue, title, 0, widgetType).setValue(value);

  }

  public void addString(String title, String value, WidgetType widgetType) {
    MAShugglebordPattern(StringValue, title, "", widgetType).setValue(value);
  }

  public void addBoolean(String title, Boolean value, WidgetType widgetType) {
    MAShugglebordPattern(BooleanValue, title, false, widgetType).setValue(value);
  }

  public boolean getBolean(String title) {

    return MAShugglebordPattern(BooleanValue, title, false).getBooleanValue(false);
  }

  public String getString(String title) {
    return MAShugglebordPattern(StringValue, title, "").getStringValue("");
  }

  public double getNum(String title) {
    return MAShugglebordPattern(NumValue, title, 0).getNumValue(0);
  }

  public boolean getBolean(String title, WidgetType widgetType) {
    return MAShugglebordPattern(BooleanValue, title, false, widgetType).getBooleanValue(false);
  }

  public String getString(String title, WidgetType widgetType) {
    return MAShugglebordPattern(StringValue, title, "", widgetType).getStringValue("");
  }

  public double getNum(String title, WidgetType widgetType) {
    return MAShugglebordPattern(NumValue, title, 0, widgetType).getNumValue(0);
  }

  public void PID(String title, MAPidController pid) {
    if (NetworkTableMap.get(title) == null) {
      PID = new ShuffleboardNetworkTableEntry(title, currnetTab);
      NetworkTableMap.put(title, PID);
    }
    PID.SetPIDValuse(pid);
  }

}