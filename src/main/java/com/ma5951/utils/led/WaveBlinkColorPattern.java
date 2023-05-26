package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

public class WaveBlinkColorPattern implements  AddressableLEDPattern{
    private Color color1;
    private Color color2;
    private Color color3;
    // private boolean color1on;
    private double interval;
    // private boolean cycle;
    private int ledLenght;
    private int colorNumber;
    private double lastChange;
    private double timestamp;

    public WaveBlinkColorPattern(Color color, Color color2, double interval) {
        colorNumber = 0;
        // color1on = true;
        // cycle = true;
        color = new Color(color.red, color.green, color.blue);
        color2 = new Color(color2.red, color2.green, color2.blue);
        this.interval = interval;
    }

    public void setParameters(Color color1, Color color2, double interval) {
        this.interval = interval;
        this.color1 = color1;
        this.color2 = color2;
    }


    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        ledLenght = buffer.getLength();

        for (double i = 0; i < ledLenght; i++) {

            if (i % interval == 0 && (i * 5) % 2 == 0) {
                colorNumber = 1;
            }
            else if (i % interval == 0 && (i * 5) % 2 != 0) {
                colorNumber = 2;
            }

            if (colorNumber == 1) {
                // color1.setLEDs(buffer);
                buffer.setLED((int)Math.round(i), color1);
            }
            else if (colorNumber == 2) {
                buffer.setLED((int)Math.round(i), color2);
            }
        }
        // cycle++;
        timestamp = Timer.getFPGATimestamp();
        if (timestamp - lastChange > 0.75){
            color3 = color1;
            color1 = color2;
            color2 = color3;
            lastChange = timestamp;
        }
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
