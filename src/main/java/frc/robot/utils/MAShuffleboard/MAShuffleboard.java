
package frc.robot.utils.MAShuffleboard;

import frc.robot.utils.Controlers.MAPidController;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

/**
 * @author yuval rader
 */

public class MAShuffleboard {
  private NetworkTableEntry NumValue;
  private NetworkTableEntry StringValue;
  private NetworkTableEntry BooleanValue;
  private Map<String, NetworkTableEntry> NetworkTableNameMap = new HashMap<String, NetworkTableEntry>();
  private String currnetTab;

  public MAShuffleboard(String tab) {
    currnetTab = tab;

  }

  public NetworkTableEntry MAShugglebordPattern(NetworkTableEntry valEntry, String title, Object DV) {
    if (NetworkTableNameMap.get(title) == null) {
      valEntry = Shuffleboard.getTab(currnetTab).add(title, DV).getEntry();
      NetworkTableNameMap.put(title, valEntry);
    }
    return NetworkTableNameMap.get(title);
  }

  public NetworkTableEntry MAShugglebordPattern(NetworkTableEntry valEntry, String title, Object DV,
      WidgetType widgetType) {
    if (NetworkTableNameMap.get(title) == null) {
      valEntry = Shuffleboard.getTab(currnetTab).add(title, DV).withWidget(widgetType).getEntry();
      NetworkTableNameMap.put(title, valEntry);
    }
    return NetworkTableNameMap.get(title);
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

    return MAShugglebordPattern(BooleanValue, title, false).getBoolean(false);
  }

  public String getString(String title) {
    return MAShugglebordPattern(StringValue, title, "").getString("");
  }

  public double getNum(String title) {
    return (double) MAShugglebordPattern(NumValue, title, 0).getNumber(0);
  }

  public boolean getBolean(String title, WidgetType widgetType) {
    return MAShugglebordPattern(BooleanValue, title, false, widgetType).getBoolean(false);
  }

  public String getString(String title, WidgetType widgetType) {
    return MAShugglebordPattern(StringValue, title, "", widgetType).getString("");
  }

  public double getNum(String title, WidgetType widgetType) {
    return (double) MAShugglebordPattern(NumValue, title, 0, widgetType).getNumber(0);
  }

  public void addPID(String title, MAPidController pid) {
    if (NetworkTableNameMap.get("setPoint") == null) {
      ShuffleboardLayout PID = Shuffleboard.getTab(currnetTab).getLayout(title, BuiltInLayouts.kList);
      NumValue = PID.add("KP", 0).getEntry();
      NetworkTableNameMap.put("KP", NumValue);
      NumValue = PID.add("KI", 0).getEntry();
      NetworkTableNameMap.put("KI", NumValue);
      NumValue = PID.add("KD", 0).getEntry();
      NetworkTableNameMap.put("KD", NumValue);
      NumValue = PID.add("setPoint", 0).getEntry();
      NetworkTableNameMap.put("setPoint", NumValue);
    }
    pid.setPID((double) NetworkTableNameMap.get("KP").getNumber(0), (double) NetworkTableNameMap.get("KI").getNumber(0),
        (double) NetworkTableNameMap.get("KD").getNumber(0));

    pid.setSetpoint((double) NetworkTableNameMap.get("setPoint").getNumber(0));
  }

}