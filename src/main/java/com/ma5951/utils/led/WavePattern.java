package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class WavePattern implements AddressableLEDPattern {
    private int numColors;
    private double period;
    private Color[] colors;

    // numColors: the number of colors to use in the wave (e.g. 3 for a three-color wave).
    // period: the period of the wave (i.e. the distance between two peaks).
    // speed: the speed at which the wave moves along the LED strip.
    // colors: an array of Color objects that define the colors to use in the wave.
    public WavePattern(int numColors, double period, double speed, Color[] colors) {
        this.numColors = numColors;
        this.period = period;
        this.colors = colors;
        setParameters(numColors, period, speed, colors);
    }

    public void setParameters(int numColors, double period, double speed, Color[] colors) {
        this.numColors = numColors;
        this.period = period;
        this.colors = colors;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        double elapsedTime = Timer.getFPGATimestamp() % period;
        double progress = elapsedTime / period;
        int numLeds = buffer.getLength();

        for (int i = 0; i < numLeds; i++) {
            double position = (double)i / (double)numLeds;
            double wavePosition = (position + progress) % 1.0;
            int colorIndex = (int)(wavePosition * numColors);
            Color currentColor = colors[colorIndex];
            buffer.setLED(i, currentColor);
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
// it looked like it worked for a brief moment then all the leds turned to the second color again