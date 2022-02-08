
package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.utils.controllers.MAPidController;

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

  private NetworkTableEntry maShufflebordPattern(NetworkTableEntry valEntry, String title, Object DV) {
    if (networkTableNameMap.get(title) == null) {
      valEntry = Shuffleboard.getTab(currnetTab).add(title, DV).getEntry();
      networkTableNameMap.put(title, valEntry); 
    }
    return networkTableNameMap.get(title);
  }

  private NetworkTableEntry maShufflebordPattern(NetworkTableEntry valEntry, String title, Object DV,
      WidgetType widgetType) {
    if (networkTableNameMap.get(title) == null) {
      valEntry = Shuffleboard.getTab(currnetTab).add(title, DV).withWidget(widgetType).getEntry();
      networkTableNameMap.put(title, valEntry);
    }
    return networkTableNameMap.get(title);
  }

  public void addNum(String title, double value) {
    maShufflebordPattern(numValue, title, 0).setValue(value);
  }

  public void addString(String title, String value) {
    maShufflebordPattern(stringValue, title, "").setValue(value);
  }

  public void addBoolean(String title, Boolean value) {
    maShufflebordPattern(booleanValue, title, false).setValue(value);
  }

  public void addNum(String title, double value, WidgetType widgetType) {
    maShufflebordPattern(numValue, title, 0, widgetType).setValue(value);
  }

  public void addString(String title, String value, WidgetType widgetType) {
    maShufflebordPattern(stringValue, title, "", widgetType).setValue(value);
  }

  public void addBoolean(String title, Boolean value, WidgetType widgetType) {
    maShufflebordPattern(booleanValue, title, false, widgetType).setValue(value);
  }

  public boolean getBolean(String title) {

    return maShufflebordPattern(booleanValue, title, false).getBoolean(false);
  }

  public String getString(String title) {
    return maShufflebordPattern(stringValue, title, "").getString("");
  }

  public double getNum(String title) {
    return (double) maShufflebordPattern(numValue, title, 0).getNumber(0);
  }

  public boolean getBolean(String title, WidgetType widgetType) {
    return maShufflebordPattern(booleanValue, title, false, widgetType).getBoolean(false);
  }

  public String getString(String title, WidgetType widgetType) {
    return maShufflebordPattern(stringValue, title, "", widgetType).getString("");
  }

  public double getNum(String title, WidgetType widgetType) {
    return (double) maShufflebordPattern(numValue, title, 0, widgetType).getNumber(0);
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