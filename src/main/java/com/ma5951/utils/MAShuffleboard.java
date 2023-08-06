package com.ma5951.utils;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class MAShuffleboard {
    private ShuffleboardTab board;
    private HashMap<String, GenericEntry> values;

    public MAShuffleboard(String tab) {
        board = Shuffleboard.getTab(tab);
        values = new HashMap<String, GenericEntry>();
    }

    public void addNum(String title, double num) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, num).getEntry());
        } else {
            values.get(title).setDouble(num);
        }
    }

    public void addString(String title, String str) {
        if(!values.containsKey(title)) {
            values.put(title, board.add(title, str).getEntry());
        } else {
            values.get(title).setString(str);
        }
    }

    public void addBoolean(String title, boolean bol) {
        if (!values.containsKey(title)) {
            values.put(title, board.add(title, bol).getEntry());
        } else {
            values.get(title).setBoolean(bol);
        }
    }
    
    public double getNum(String title) {
        if (values.containsKey(title)) {
            return values.get(title).getDouble(0);
        }
        System.err.println("none existing title: " + title);
        return 0;
    }

    public String getString(String title) {

        if (values.containsKey(title)) {
            return values.get(title).getString("null");
        }
        System.err.println("none existing title: " + title);
        return "null";
    }

    public Boolean getBoolean(String title) {

        if (values.containsKey(title)) {
            return values.get(title).getBoolean(false);
        }
        System.err.println("none existing title: " + title);
        return false;
    }

    public pidControllerGainSupplier getPidControllerGainSupplier(
        double KP, double KI, double KD) {
            return new pidControllerGainSupplier(this, KP, KI, KD);
    }

    public pidControllerGainSupplier getPidControllerGainSupplier() {
        return this.getPidControllerGainSupplier(0, 0, 0);
    }

    public class pidControllerGainSupplier {
        
        private static final String KP_STRING = "KP";
        private static final String KI_STRING = "KI";
        private static final String KD_STRING = "KD";
        private MAShuffleboard shuffleboard;

        private pidControllerGainSupplier(MAShuffleboard shuffleboard,
         double KP, double KI, double KD) {
            this.shuffleboard = shuffleboard;
            shuffleboard.addNum(KP_STRING, KP);
            shuffleboard.addNum(KI_STRING, KI);
            shuffleboard.addNum(KD_STRING, KD);
        }

        public double getKP() {
            return shuffleboard.getNum(KP_STRING);
        }

        public double getKI() {
            return shuffleboard.getNum(KI_STRING);
        }

        public double getKD() {
            return shuffleboard.getNum(KD_STRING);
        }
    }
}