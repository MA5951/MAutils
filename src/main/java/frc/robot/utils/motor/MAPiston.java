// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.motor;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class MAPiston {
    private Solenoid solenoid;
    private DoubleSolenoid doubleSolenoid;
    private boolean isDoubleSolenoid;

    public MAPiston(int channel) {
        isDoubleSolenoid = false;
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, channel);
    }

    public MAPiston(int forwardChannel, int reverseChannel) {
        isDoubleSolenoid = true;
        doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
    }

    public void set(boolean on) {
        if (isDoubleSolenoid) {
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
        if (isDoubleSolenoid)
            return fromValueToBoolean(doubleSolenoid.get());
        else
            return solenoid.get();
    }

    public void toggle() {
        if (isDoubleSolenoid)
            doubleSolenoid.set(fromBooleanToValue(!get()));
        else
            solenoid.set(!get());
    }

}