package com.ma5951.utils.fallbacks;

import javax.xml.validation.Validator;

public class Sensor_Fallbacks {

    public static boolean allSensorChecks(double value, double lastValue, 
                                          double threshold, double maxValue, 
                                          double minValue)
    {
        return  driftCheck(value, lastValue, threshold) ||
                inRangeCheck(value, maxValue, minValue);
    }

    public static boolean driftCheck(double value, double lastValue, double threshold)
    {
        return value - lastValue >= threshold || value + lastValue <= threshold;
    }

    public static boolean inRangeCheck(double value, double maxValue, double minValue)
    {
        return value > maxValue || value < minValue;
    }

}
