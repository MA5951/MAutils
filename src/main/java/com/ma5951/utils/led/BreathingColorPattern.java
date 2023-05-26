package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class BreathingColorPattern implements  AddressableLEDPattern{
    public Color pattern = new Color(0, 0, 0);
    public SolidColorPattern color;

    private double interval;
    private boolean direction;

    private double originalRed;
    private double originalGreen;
    private double originalBlue;

    public BreathingColorPattern(Color color, double interval) {
        setParameters(color, interval);
        this.color = new SolidColorPattern(color);
    }

    public void setParameters(Color color, double interval) {
        this.interval = interval;
        originalBlue = color.blue;
        originalGreen = color.green;
        originalRed = color.red;
        color = new Color(color.red, color.green, color.blue);
        pattern = new Color(color.red, color.green, color.blue);
        direction = true;
    }



    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        if (direction) {
            if (pattern.red > 0) {
                pattern = new Color(pattern.red - (originalRed / (interval/0.05)), pattern.green, pattern.blue);
            }
            if (pattern.green > 0) {
                pattern = new Color(pattern.red, pattern.green - (originalGreen / (interval/0.05)), pattern.blue);
            }
            if (pattern.blue > 0) {
                pattern = new Color(pattern.red, pattern.green, pattern.blue - (originalBlue / (interval/0.05)));
            }
            if (pattern.red <= 4.9E-4 && pattern.green <= 4.9E-4 && pattern.blue <= 4.9E-4) {
                direction = false;
            }
            color = new SolidColorPattern(pattern);
        }
        else {
            if (pattern.red < originalRed) {
                pattern = new Color(pattern.red + (originalRed / (interval/0.05)), pattern.green, pattern.blue);
            }
            if (pattern.green < originalGreen) {
                pattern = new Color(pattern.red, pattern.green + (originalGreen / (interval/0.05)), pattern.blue);
            }
            if (pattern.blue < originalBlue) {
                pattern = new Color(pattern.red, pattern.green, pattern.blue + (originalBlue / (interval/0.05)));
            }
            if (pattern.red >= originalRed && pattern.green >= originalGreen && pattern.blue >= originalBlue) {
                direction = true;
            }
            color = new SolidColorPattern(pattern);
        }
        color.setLEDs(buffer);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}