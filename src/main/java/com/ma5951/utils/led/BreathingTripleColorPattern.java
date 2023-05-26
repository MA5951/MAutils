package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class BreathingTripleColorPattern implements  AddressableLEDPattern{
    public Color pattern = new Color(0, 0, 0);
    public SolidColorPattern color;
    private double interval;
    private int direction;

    private double originalRed;
    private double originalGreen;
    private double originalBlue;

    private double originalRed2;
    private double originalGreen2;
    private double originalBlue2;

    public BreathingTripleColorPattern(Color color, Color color2, double interval) {
        setParameters(color, color2, interval);
        this.color = new SolidColorPattern(color);
    }

    public void setParameters(Color color, Color color2, double interval) {
        this.interval = interval;
        originalBlue = color.blue;
        originalGreen = color.green;
        originalRed = color.red;

        originalBlue2 = color2.blue;
        originalGreen2 = color2.green;
        originalRed2 = color2.red;

        color = new Color(originalRed, originalGreen, originalBlue);
        pattern = new Color(originalRed, originalGreen, originalBlue);

        direction = 0;
    }



    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        if (direction == 0) {
            //from color to black
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
                direction = 1;
            }

            color = new SolidColorPattern(pattern);
        }
        else if (direction == 1){
            // from black to color2
            if (pattern.red < originalRed2) {
                pattern = new Color(pattern.red + (originalRed2 / (interval/0.05)), pattern.green, pattern.blue);
            }
            if (pattern.green < originalGreen2) {
                pattern = new Color(pattern.red, pattern.green + (originalGreen2 / (interval/0.05)), pattern.blue);
            }
            if (pattern.blue < originalBlue2) {
                pattern = new Color(pattern.red, pattern.green, pattern.blue + (originalBlue2 / (interval/0.05)));
            }
            if (pattern.red >= originalRed2 && pattern.green >= originalGreen2 && pattern.blue >= originalBlue2) {
                direction = 2;
            }
            color = new SolidColorPattern(pattern);
        }
        else if (direction == 2){
            // from color2 to black
            if (pattern.red > 0) {
                pattern = new Color(pattern.red - (originalRed2 / (interval/0.05)), pattern.green, pattern.blue);
            }
            if (pattern.green > 0) {
                pattern = new Color(pattern.red, pattern.green - (originalGreen2 / (interval/0.05)), pattern.blue);
            }
            if (pattern.blue > 0) {
                pattern = new Color(pattern.red, pattern.green, pattern.blue - (originalBlue2 / (interval/0.05)));
            }
            if (pattern.red <= 4.9E-4 && pattern.green <= 4.9E-4 && pattern.blue <= 4.9E-4) {
                direction = 3;
            }
            color = new SolidColorPattern(pattern);
        }
        else if (direction == 3){
            // from black to color
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
                direction = 0;
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
