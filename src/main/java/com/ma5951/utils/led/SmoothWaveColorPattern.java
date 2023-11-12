package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class SmoothWaveColorPattern implements AddressableLEDPattern {
    private int numColors;
    private double period;
    private double speed;
    private Color[] colors;

    public SmoothWaveColorPattern(int numColors, double period, double speed, Color[] colors) {
        this.numColors = numColors;
        this.period = period;
        this.speed = speed;
        this.colors = colors;
        setParameters(numColors, period, speed, colors);
    }

    public void setParameters(int numColors, double period, double speed, Color[] colors) {
        this.numColors = numColors;
        this.period = period;
        this.speed = speed;
        this.colors = colors.clone();
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double elapsedTime = Timer.getFPGATimestamp();
        for (int i = 0; i < buffer.getLength(); i++) {
            double position = ((double) i / buffer.getLength()) + (elapsedTime * speed / period);
            double progress = position - (int) position;

            int startColorIndex = (int) (position % numColors);
            int endColorIndex = (startColorIndex + 1) % numColors;
            Color startColor = colors[startColorIndex];
            Color endColor = colors[endColorIndex];

            Color currentColor = new Color(
                    startColor.red + (endColor.red - startColor.red) * progress,
                    startColor.green + (endColor.green - startColor.green) * progress,
                    startColor.blue + (endColor.blue - startColor.blue) * progress
            );

            buffer.setLED(i, currentColor);
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
