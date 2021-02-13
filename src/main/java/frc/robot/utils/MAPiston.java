// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class MAPiston {
    private Solenoid solenoid;
    private DoubleSolenoid doubleSolenoid;
    private boolean IsDoubleSolenoid;

    public MAPiston(int channel) {
        IsDoubleSolenoid = false;
        solenoid = new Solenoid(channel);
    }

    public MAPiston(int forwardChannel, int reverseChannel) {
        IsDoubleSolenoid = true;
        doubleSolenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
    }

    public void set(boolean on) {
        if (IsDoubleSolenoid) {
            doubleSolenoid.set(FromBooleanToValue(on));
        } else {
            solenoid.set(on);
        }
    }

    private Value FromBooleanToValue(boolean value) {
        if (value)
            return Value.kForward;
        else
            return Value.kReverse;
    }

    private boolean FromValueToBoolean(Value value) {
        if (value == Value.kForward)
            return true;
        else
            return false;
    }

    public boolean get() {
        if (IsDoubleSolenoid)
            return FromValueToBoolean(doubleSolenoid.get());
        else
            return solenoid.get();
    }

    public void Toggle() {
        if (IsDoubleSolenoid)
            doubleSolenoid.toggle();
        else
            solenoid.toggle();
    }

}
