
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
  private NetworkTableEntry numValue;
  private NetworkTableEntry stringValue;
  private NetworkTableEntry booleanValue;
  private Map<String, NetworkTableEntry> networkTableNameMap = new HashMap<String, NetworkTableEntry>();
  private String currnetTab;

  public MAShuffleboard(String tab) {
    currnetTab = tab;

  }

  public NetworkTableEntry maShugglebordPattern(NetworkTableEntry valEntry, String title, Object DV) {
    if (networkTableNameMap.get(title) == null) {
      valEntry = Shuffleboard.getTab(currnetTab).add(title, DV).getEntry();
      networkTableNameMap.put(title, valEntry);
    }
    return networkTableNameMap.get(title);
  }

  public NetworkTableEntry maShugglebordPattern(NetworkTableEntry valEntry, String title, Object DV,
      WidgetType widgetType) {
    if (networkTableNameMap.get(title) == null) {
      valEntry = Shuffleboard.getTab(currnetTab).add(title, DV).withWidget(widgetType).getEntry();
      networkTableNameMap.put(title, valEntry);
    }
    return networkTableNameMap.get(title);
  }

  public void addNum(String title, double value) {
    maShugglebordPattern(numValue, title, 0).setValue(value);
  }

  public void addString(String title, String value) {
    maShugglebordPattern(stringValue, title, "").setValue(value);
  }

  public void addBoolean(String title, Boolean value) {
    maShugglebordPattern(booleanValue, title, false).setValue(value);
  }

  public void addNum(String title, double value, WidgetType widgetType) {
    maShugglebordPattern(numValue, title, 0, widgetType).setValue(value);
  }

  public void addString(String title, String value, WidgetType widgetType) {
    maShugglebordPattern(stringValue, title, "", widgetType).setValue(value);
  }

  public void addBoolean(String title, Boolean value, WidgetType widgetType) {
    maShugglebordPattern(booleanValue, title, false, widgetType).setValue(value);
  }

  public boolean getBolean(String title) {

    return maShugglebordPattern(booleanValue, title, false).getBoolean(false);
  }

  public String getString(String title) {
    return maShugglebordPattern(stringValue, title, "").getString("");
  }

  public double getNum(String title) {
    return (double) maShugglebordPattern(numValue, title, 0).getNumber(0);
  }

  public boolean getBolean(String title, WidgetType widgetType) {
    return maShugglebordPattern(booleanValue, title, false, widgetType).getBoolean(false);
  }

  public String getString(String title, WidgetType widgetType) {
    return maShugglebordPattern(stringValue, title, "", widgetType).getString("");
  }

  public double getNum(String title, WidgetType widgetType) {
    return (double) maShugglebordPattern(numValue, title, 0, widgetType).getNumber(0);
  }

  public void addPID(String title, MAPidController pid) {
    if (networkTableNameMap.get("setPoint") == null) {
      ShuffleboardLayout PID = Shuffleboard.getTab(currnetTab).getLayout(title, BuiltInLayouts.kList);
      numValue = PID.add("KP", 0).getEntry();
      networkTableNameMap.put("KP", numValue);
      numValue = PID.add("KI", 0).getEntry();
      networkTableNameMap.put("KI", numValue);
      numValue = PID.add("KD", 0).getEntry();
      networkTableNameMap.put("KD", numValue);
      numValue = PID.add("setPoint", 0).getEntry();
      networkTableNameMap.put("setPoint", numValue);
    }
    pid.setPID((double) networkTableNameMap.get("KP").getNumber(0), (double) networkTableNameMap.get("KI").getNumber(0),
        (double) networkTableNameMap.get("KD").getNumber(0));

    pid.setSetpoint((double) networkTableNameMap.get("setPoint").getNumber(0));
  }

}