package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowColorPattern implements AddressableLEDPattern {
    private int firstHue = 0;

    public RainbowColorPattern() {

    }

    /**
     * @param buffer
     */
    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        int currentHue;
        int length = buffer.getLength();
        for (int i = 0; i < length; i++) {
            currentHue = (firstHue + (i * 180 / length)) % 180;
            buffer.setHSV(i, currentHue, 255, 128);
        }

        firstHue = (firstHue + 3) % 180;
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
