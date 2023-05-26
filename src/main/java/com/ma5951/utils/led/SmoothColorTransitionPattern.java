package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class SmoothColorTransitionPattern implements AddressableLEDPattern {
    private Color startColor;
    private Color endColor;
    private double transitionTime;

    public SmoothColorTransitionPattern(Color startColor, Color endColor, double transitionTime) {
        this.startColor = startColor;
        this.endColor = endColor;
        this.transitionTime = transitionTime;
        setParameters(startColor, endColor, transitionTime);
    }

    public void setParameters(Color startColor, Color endColor, double transitionTime) {
        this.startColor = new Color(startColor.red, startColor.green, startColor.blue);
        this.endColor = new Color(endColor.red, endColor.green, endColor.blue);
        this.transitionTime = transitionTime;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double elapsedTime = Timer.getFPGATimestamp() % (2 * transitionTime);
        double progress = elapsedTime / transitionTime;

        Color currentColor;

        if (progress <= 1) {
            currentColor = new Color(
                startColor.red + (endColor.red - startColor.red) * progress,
                startColor.green + (endColor.green - startColor.green) * progress,
                startColor.blue + (endColor.blue - startColor.blue) * progress
            );
        } else {
            currentColor = new Color(
                endColor.red + (startColor.red - endColor.red) * (progress - 1),
                endColor.green + (startColor.green - endColor.green) * (progress - 1),
                endColor.blue + (startColor.blue - endColor.blue) * (progress - 1)
            );
        }

        AddressableLEDPattern colorPattern = new SolidColorPattern(currentColor);
        colorPattern.setLEDs(buffer);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
