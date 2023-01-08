
package com.ma5951.utils;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.Subscriber;

public class MAShuffleboard {
    private Map<String, Subscriber> networkSubscriberTableNameMap = new HashMap<String, Subscriber>();
    private Map<String, Publisher> networkPublisherTableNameMap = new HashMap<String, Publisher>();

    private NetworkTableInstance inst;
    private NetworkTable dataTable;

    public MAShuffleboard(String tab) {
        inst = NetworkTableInstance.getDefault();
        dataTable = inst.getTable("SmartDashboard").getSubTable(tab);
    }

    public void addNum(String title, double value) {
        if (!networkPublisherTableNameMap.containsKey(title)) {
            Publisher pub = dataTable.getDoubleTopic(title).publish();
  
            networkPublisherTableNameMap.put(title, pub);
        }
        DoublePublisher pub = (DoublePublisher) networkPublisherTableNameMap.get(title);
        pub.set(value);
    }

    public void addString(String title, String value) {
        if (!networkPublisherTableNameMap.containsKey(title)) {
            Publisher pub = dataTable.getDoubleTopic(title).publish();
  
            networkPublisherTableNameMap.put(title, pub);
        }
        StringPublisher pub = (StringPublisher) networkPublisherTableNameMap.get(title);
        pub.set(value);
    }

    public void addBoolean(String title, Boolean value) {
        if (!networkPublisherTableNameMap.containsKey(title)) {
            Publisher pub = dataTable.getDoubleTopic(title).publish();
  
            networkPublisherTableNameMap.put(title, pub);
        }
        BooleanPublisher pub = (BooleanPublisher) networkPublisherTableNameMap.get(title);
        pub.set(value);
    }

    public boolean getBoolean(String title, boolean value) {
        if (!networkSubscriberTableNameMap.containsKey(title)) {
            Subscriber sub = dataTable.getBooleanTopic(title).subscribe(value);
  
            networkSubscriberTableNameMap.put(title, sub);
        }
        BooleanSubscriber sub = (BooleanSubscriber) networkSubscriberTableNameMap.get(title);
        return sub.get();
    }

    public String getString(String title, String value) {
        if (!networkSubscriberTableNameMap.containsKey(title)) {
            Subscriber sub = dataTable.getStringTopic(title).subscribe(value);
  
            networkSubscriberTableNameMap.put(title, sub);
        }
        StringSubscriber sub = (StringSubscriber) networkSubscriberTableNameMap.get(title);
        return sub.get();
    }

    public double getNum(String title, double value) {
        if (!networkSubscriberTableNameMap.containsKey(title)) {
            Subscriber sub = dataTable.getDoubleTopic(title).subscribe(value);
  
            networkSubscriberTableNameMap.put(title, sub);
        }
        DoubleSubscriber sub = (DoubleSubscriber) networkSubscriberTableNameMap.get(title);
        return sub.get();
    }

    public boolean getBoolean(String title) {
        return getBoolean(title, false);
    }

    public String getString(String title) {
        return getString(title, "Why?");
    }

    public double getNum(String title) {
        return getNum(title, 0);
    }
}