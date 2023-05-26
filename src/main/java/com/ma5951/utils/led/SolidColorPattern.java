// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class SolidColorPattern implements AddressableLEDPattern {

    Color color;

    public SolidColorPattern(Color color) {
        this.color = color;
    }

    public void setColor(Color color) {
        this.color = color;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }

}
