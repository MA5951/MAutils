package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class BlinkingColorPattern implements  AddressableLEDPattern{
    private SolidColorPattern onPattern;
    private SolidColorPattern offPattern;
    private double interval;
    private boolean on;
    private double lastChange;

    public BlinkingColorPattern(Color onColor, Color offColor, double interval) {
        onPattern = new SolidColorPattern(onColor);
        offPattern = new SolidColorPattern(offColor);
        this.interval = interval;
    }

    public void setParameters(Color color1, Color color2, double interval) {
        this.interval = interval;
        this.onPattern.setColor(color1);
        this.offPattern.setColor(color2);
    }



    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double timestamp = Timer.getFPGATimestamp();
        if (timestamp - lastChange > interval) {
            on = !on;
            lastChange = timestamp;
        }
        if (on) {
            onPattern.setLEDs(buffer);
        }
        else {
            offPattern.setLEDs(buffer);
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
