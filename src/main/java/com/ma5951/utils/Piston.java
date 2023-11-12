// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class Piston {
    private Solenoid solenoid;
    private DoubleSolenoid doubleSolenoid;

    public Piston(int channel) {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, channel);
    }

    public Piston(int forwardChannel, int reverseChannel) {
       doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
    }

    public void set(boolean on) {
        if (solenoid == null) {
            doubleSolenoid.set(fromBooleanToValue(on));
        } else {
            solenoid.set(on);
        }
    }

    private Value fromBooleanToValue(boolean value) {
        if (value)
            return Value.kForward;
        else
            return Value.kReverse;
    }

    private boolean fromValueToBoolean(Value value) {
        if (value == Value.kForward)
            return true;
        else
            return false;
    }

    public boolean get() {
        if (solenoid == null)
            return fromValueToBoolean(doubleSolenoid.get());
        else
            return solenoid.get();
    }

    public void toggle() {
        if (solenoid == null)
            doubleSolenoid.set(fromBooleanToValue(!get()));
        else
            solenoid.set(!get());
    }

    public void off() {
        if (solenoid == null)
            doubleSolenoid.set(Value.kOff);
        else
            solenoid.set(false);
    }
}