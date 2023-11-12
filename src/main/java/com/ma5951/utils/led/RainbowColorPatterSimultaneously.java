package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowColorPatterSimultaneously implements AddressableLEDPattern {
    private int firstHue = 0;

    public RainbowColorPatterSimultaneously() {

    }

    /**
     * @param buffer
     */
    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        int currentHue;
        int length = buffer.getLength();

        // rainbow of all LEDs at once
        for (int i = 0; i <= 180; i++) {
            currentHue = (firstHue + (i / length)) % 180;
            for (int j = 0; j < length; j++) {
                buffer.setHSV(j, currentHue, 255, 128);
            }
        }
        

        firstHue = (firstHue + 3) % 180;
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
